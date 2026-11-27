/**
 * @file slam_pose_graph.c
 * @brief 纯 C99 2D 位姿图优化器实现（LM + Huber 核 + 无矩阵块雅可比 PCG）。
 */
#include "slam_pose_graph.h"
#include <string.h>

/** @brief 0 号锚定节点先验权重（固定世界坐标系原点）。 */
#define ANCHOR_WEIGHT 1e6f

SlamGraphConfig SlamPoseGraph_DefaultConfig(void)
{
    SlamGraphConfig cfg;
    cfg.max_lm_iterations = 15;
    cfg.max_pcg_iterations = 120;
    cfg.huber_delta = 2.0f;
    cfg.init_lambda = 1e-2f;
    cfg.step_tol = 1e-4f;
    return cfg;
}

/**
 * @brief 计算单条约束边的误差向量 e_ij 及对节点 i、j 的 3x3 雅可比矩阵 A、B。
 */
static void edge_error_and_jacobians(Pose pi, Pose pj, Pose meas,
                                     float e[3], float A[3][3], float B[3][3])
{
    float dx = pj.x - pi.x;
    float dy = pj.y - pi.y;
    float c = cosf(pi.theta);
    float s = sinf(pi.theta);

    e[0] = c * dx + s * dy - meas.x;
    e[1] = -s * dx + c * dy - meas.y;
    e[2] = Slam_NormalizeAngle(pj.theta - pi.theta - meas.theta);

    if (A) {
        A[0][0] = -c;  A[0][1] = -s;  A[0][2] = -s * dx + c * dy;
        A[1][0] =  s;  A[1][1] = -c;  A[1][2] = -c * dx - s * dy;
        A[2][0] = 0.0f; A[2][1] = 0.0f; A[2][2] = -1.0f;
    }
    if (B) {
        B[0][0] =  c;  B[0][1] =  s;  B[0][2] = 0.0f;
        B[1][0] = -s;  B[1][1] =  c;  B[1][2] = 0.0f;
        B[2][0] = 0.0f; B[2][1] = 0.0f; B[2][2] = 1.0f;
    }
}

/**
 * @brief 计算单条边的 Huber 鲁棒权重与加权平方代价。
 */
static float edge_robust_weight(const SlamGraphEdge *edge, const float e[3],
                                float huber_delta, float *out_cost)
{
    float q = edge->info_xy * (e[0] * e[0] + e[1] * e[1]) +
              edge->info_theta * (e[2] * e[2]);
    if (!edge->is_loop || huber_delta <= 0.0f) {
        if (out_cost) *out_cost = 0.5f * q;
        return 1.0f;
    }
    float r = sqrtf(fmaxf(0.0f, q));
    if (r <= huber_delta) {
        if (out_cost) *out_cost = 0.5f * q;
        return 1.0f;
    }
    if (out_cost) *out_cost = huber_delta * (r - 0.5f * huber_delta);
    return huber_delta / r;
}

/**
 * @brief 评估当前位姿集合的总鲁棒加权代价。
 */
static float evaluate_total_cost(const Pose *poses, uint16_t num_nodes,
                                 const SlamGraphEdge *edges, uint16_t num_edges,
                                 float huber_delta)
{
    float total = 0.0f;
    for (uint16_t k = 0; k < num_edges; ++k) {
        const SlamGraphEdge *edge = &edges[k];
        if (edge->from >= num_nodes || edge->to >= num_nodes) continue;
        float e[3];
        edge_error_and_jacobians(poses[edge->from], poses[edge->to], edge->measurement,
                                 e, NULL, NULL);
        float c = 0.0f;
        (void)edge_robust_weight(edge, e, huber_delta, &c);
        total += c;
    }
    return total;
}

/**
 * @brief 对 3x3 对称正定对角块求逆（伴随矩阵法），失败时退回对角逆。
 */
static void invert_3x3(const float M[9], float inv[9])
{
    float a00 = M[0], a01 = M[1], a02 = M[2];
    float a11 = M[4], a12 = M[5], a22 = M[8];

    float c00 = a11 * a22 - a12 * a12;
    float c01 = a02 * a12 - a01 * a22;
    float c02 = a01 * a12 - a02 * a11;
    float c11 = a00 * a22 - a02 * a02;
    float c12 = a01 * a02 - a00 * a12;
    float c22 = a00 * a11 - a01 * a01;
    float det = a00 * c00 + a01 * c01 + a02 * c02;

    if (fabsf(det) < 1e-12f) {
        memset(inv, 0, 9 * sizeof(float));
        inv[0] = (fabsf(a00) > 1e-8f) ? (1.0f / a00) : 1.0f;
        inv[4] = (fabsf(a11) > 1e-8f) ? (1.0f / a11) : 1.0f;
        inv[8] = (fabsf(a22) > 1e-8f) ? (1.0f / a22) : 1.0f;
        return;
    }
    float id = 1.0f / det;
    inv[0] = c00 * id; inv[1] = c01 * id; inv[2] = c02 * id;
    inv[3] = c01 * id; inv[4] = c11 * id; inv[5] = c12 * id;
    inv[6] = c02 * id; inv[7] = c12 * id; inv[8] = c22 * id;
}

/**
 * @brief 无矩阵计算 y = (H + lambda * I + Anchor_0) * p。
 */
static void apply_hessian(const Pose *poses, uint16_t num_nodes,
                          const SlamGraphEdge *edges, uint16_t num_edges,
                          float huber_delta, float lambda,
                          const float *p, float *y)
{
    int dim = (int)num_nodes * 3;
    for (int idx = 0; idx < dim; ++idx) y[idx] = lambda * p[idx];
    // 锚定 0 号节点
    y[0] += ANCHOR_WEIGHT * p[0];
    y[1] += ANCHOR_WEIGHT * p[1];
    y[2] += ANCHOR_WEIGHT * p[2];

    for (uint16_t k = 0; k < num_edges; ++k) {
        const SlamGraphEdge *edge = &edges[k];
        if (edge->from >= num_nodes || edge->to >= num_nodes) continue;
        float e[3], A[3][3], B[3][3];
        edge_error_and_jacobians(poses[edge->from], poses[edge->to], edge->measurement, e, A, B);
        float w = edge_robust_weight(edge, e, huber_delta, NULL);
        const float *pi = &p[edge->from * 3U];
        const float *pj = &p[edge->to * 3U];

        // v = A * p_i + B * p_j
        float v[3];
        for (int r = 0; r < 3; ++r) {
            v[r] = A[r][0] * pi[0] + A[r][1] * pi[1] + A[r][2] * pi[2] +
                   B[r][0] * pj[0] + B[r][1] * pj[1] + B[r][2] * pj[2];
        }
        // u = w * Omega * v
        float u[3] = {
            w * edge->info_xy * v[0],
            w * edge->info_xy * v[1],
            w * edge->info_theta * v[2]
        };
        float *yi = &y[edge->from * 3U];
        float *yj = &y[edge->to * 3U];
        for (int c = 0; c < 3; ++c) {
            yi[c] += A[0][c] * u[0] + A[1][c] * u[1] + A[2][c] * u[2];
            yj[c] += B[0][c] * u[0] + B[1][c] * u[1] + B[2][c] * u[2];
        }
    }
}

/**
 * @brief 组装右端项 -g 与各节点 3x3 对角块逆矩阵（块雅可比预条件器）。
 */
static void build_rhs_and_precond(const Pose *poses, uint16_t num_nodes,
                                  const SlamGraphEdge *edges, uint16_t num_edges,
                                  float huber_delta, float lambda,
                                  SlamGraphWorkspace *ws)
{
    int dim = (int)num_nodes * 3;
    memset(ws->grad, 0, (size_t)dim * sizeof(float));
    memset(ws->diag_inv, 0, (size_t)num_nodes * 9U * sizeof(float));

    for (uint16_t i = 0; i < num_nodes; ++i) {
        float *D = &ws->diag_inv[i * 9U];
        D[0] = D[4] = D[8] = lambda;
    }
    ws->diag_inv[0] += ANCHOR_WEIGHT;
    ws->diag_inv[4] += ANCHOR_WEIGHT;
    ws->diag_inv[8] += ANCHOR_WEIGHT;

    for (uint16_t k = 0; k < num_edges; ++k) {
        const SlamGraphEdge *edge = &edges[k];
        if (edge->from >= num_nodes || edge->to >= num_nodes) continue;
        float e[3], A[3][3], B[3][3];
        edge_error_and_jacobians(poses[edge->from], poses[edge->to], edge->measurement, e, A, B);
        float w = edge_robust_weight(edge, e, huber_delta, NULL);
        float om[3] = {w * edge->info_xy, w * edge->info_xy, w * edge->info_theta};

        float *gi = &ws->grad[edge->from * 3U];
        float *gj = &ws->grad[edge->to * 3U];
        float *Di = &ws->diag_inv[edge->from * 9U];
        float *Dj = &ws->diag_inv[edge->to * 9U];

        for (int c = 0; c < 3; ++c) {
            // 右端项为 -g = -J^T * w * Omega * e
            gi[c] -= A[0][c] * om[0] * e[0] + A[1][c] * om[1] * e[1] + A[2][c] * om[2] * e[2];
            gj[c] -= B[0][c] * om[0] * e[0] + B[1][c] * om[1] * e[1] + B[2][c] * om[2] * e[2];
            for (int d = 0; d < 3; ++d) {
                Di[c * 3 + d] += A[0][c] * om[0] * A[0][d] +
                                 A[1][c] * om[1] * A[1][d] +
                                 A[2][c] * om[2] * A[2][d];
                Dj[c * 3 + d] += B[0][c] * om[0] * B[0][d] +
                                 B[1][c] * om[1] * B[1][d] +
                                 B[2][c] * om[2] * B[2][d];
            }
        }
    }

    // 原地将每个节点的 3x3 对角块求逆
    for (uint16_t i = 0; i < num_nodes; ++i) {
        float tmp[9];
        memcpy(tmp, &ws->diag_inv[i * 9U], sizeof(tmp));
        invert_3x3(tmp, &ws->diag_inv[i * 9U]);
    }
}

/**
 * @brief 应用块雅可比预条件器：z = D^{-1} * r。
 */
static void apply_precond(const float *diag_inv, uint16_t num_nodes,
                          const float *r, float *z)
{
    for (uint16_t i = 0; i < num_nodes; ++i) {
        const float *M = &diag_inv[i * 9U];
        const float *ri = &r[i * 3U];
        float *zi = &z[i * 3U];
        zi[0] = M[0] * ri[0] + M[1] * ri[1] + M[2] * ri[2];
        zi[1] = M[3] * ri[0] + M[4] * ri[1] + M[5] * ri[2];
        zi[2] = M[6] * ri[0] + M[7] * ri[1] + M[8] * ri[2];
    }
}

/**
 * @brief 预条件共轭梯度法求解 (H + lambda * I) * delta = rhs。
 */
static void solve_pcg(const Pose *poses, uint16_t num_nodes,
                      const SlamGraphEdge *edges, uint16_t num_edges,
                      float huber_delta, float lambda, int max_iter,
                      SlamGraphWorkspace *ws)
{
    int dim = (int)num_nodes * 3;
    memset(ws->delta, 0, (size_t)dim * sizeof(float));
    memcpy(ws->pcg_r, ws->grad, (size_t)dim * sizeof(float));
    apply_precond(ws->diag_inv, num_nodes, ws->pcg_r, ws->pcg_z);
    memcpy(ws->pcg_p, ws->pcg_z, (size_t)dim * sizeof(float));

    double rz_old = 0.0, r0_norm = 0.0;
    for (int i = 0; i < dim; ++i) {
        rz_old += (double)ws->pcg_r[i] * (double)ws->pcg_z[i];
        r0_norm += (double)ws->pcg_r[i] * (double)ws->pcg_r[i];
    }
    if (r0_norm < 1e-16) return;
    double tol_sq = r0_norm * 1e-10;

    for (int iter = 0; iter < max_iter; ++iter) {
        apply_hessian(poses, num_nodes, edges, num_edges, huber_delta, lambda,
                      ws->pcg_p, ws->pcg_ap);
        double pAp = 0.0;
        for (int i = 0; i < dim; ++i) pAp += (double)ws->pcg_p[i] * (double)ws->pcg_ap[i];
        if (pAp <= 1e-18) break;

        double alpha = rz_old / pAp;
        double r_norm = 0.0;
        for (int i = 0; i < dim; ++i) {
            ws->delta[i] += (float)(alpha * (double)ws->pcg_p[i]);
            ws->pcg_r[i] -= (float)(alpha * (double)ws->pcg_ap[i]);
            r_norm += (double)ws->pcg_r[i] * (double)ws->pcg_r[i];
        }
        if (r_norm < tol_sq) break;

        apply_precond(ws->diag_inv, num_nodes, ws->pcg_r, ws->pcg_z);
        double rz_new = 0.0;
        for (int i = 0; i < dim; ++i) rz_new += (double)ws->pcg_r[i] * (double)ws->pcg_z[i];
        if (fabs(rz_old) < 1e-18) break;
        double beta = rz_new / rz_old;
        for (int i = 0; i < dim; ++i) {
            ws->pcg_p[i] = ws->pcg_z[i] + (float)(beta * (double)ws->pcg_p[i]);
        }
        rz_old = rz_new;
    }
}

bool SlamPoseGraph_Optimize(Pose *poses, uint16_t num_nodes,
                            const SlamGraphEdge *edges, uint16_t num_edges,
                            const SlamGraphConfig *cfg, SlamGraphWorkspace *ws,
                            float *out_final_cost)
{
    if (!poses || !edges || !ws || num_nodes < 2U || num_edges == 0U ||
        num_nodes > ws->max_nodes || !ws->diag_inv || !ws->grad || !ws->delta ||
        !ws->pcg_r || !ws->pcg_z || !ws->pcg_p || !ws->pcg_ap || !ws->trial_poses) {
        return false;
    }

    SlamGraphConfig c = cfg ? *cfg : SlamPoseGraph_DefaultConfig();
    float lambda = c.init_lambda;
    float current_cost = evaluate_total_cost(poses, num_nodes, edges, num_edges, c.huber_delta);

    for (int iter = 0; iter < c.max_lm_iterations; ++iter) {
        build_rhs_and_precond(poses, num_nodes, edges, num_edges, c.huber_delta, lambda, ws);
        solve_pcg(poses, num_nodes, edges, num_edges, c.huber_delta, lambda,
                  c.max_pcg_iterations, ws);

        // 生成试探步位姿并检查最大步长
        float max_step = 0.0f;
        bool finite_ok = true;
        for (uint16_t i = 0; i < num_nodes; ++i) {
            float dx = ws->delta[i * 3U + 0U];
            float dy = ws->delta[i * 3U + 1U];
            float dth = ws->delta[i * 3U + 2U];
            if (!isfinite(dx) || !isfinite(dy) || !isfinite(dth)) {
                finite_ok = false;
                break;
            }
            ws->trial_poses[i].x = poses[i].x + dx;
            ws->trial_poses[i].y = poses[i].y + dy;
            ws->trial_poses[i].theta = Slam_NormalizeAngle(poses[i].theta + dth);
            float s = fmaxf(hypotf(dx, dy), fabsf(dth));
            if (s > max_step) max_step = s;
        }
        // 保持 0 号锚定节点绝对不动
        ws->trial_poses[0] = poses[0];

        float trial_cost = finite_ok
                               ? evaluate_total_cost(ws->trial_poses, num_nodes, edges,
                                                     num_edges, c.huber_delta)
                               : 1e30f;

        if (finite_ok && trial_cost <= current_cost + 1e-6f) {
            memcpy(poses, ws->trial_poses, (size_t)num_nodes * sizeof(Pose));
            current_cost = trial_cost;
            lambda = fmaxf(1e-5f, lambda * 0.35f);
            if (max_step < c.step_tol) break;
        } else {
            lambda = fminf(1e4f, lambda * 5.0f);
        }
    }

    if (out_final_cost) *out_final_cost = current_cost;
    return true;
}
