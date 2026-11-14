# -*- coding: utf-8 -*-
"""Extract plain text (page by page) from the N10 docs into UTF-8 .txt files.

Run with the bundled interpreter:
  C:\\Users\\wyp20\\.dsh\\dsh-runtimes\\dsh-primary-runtime\\dependencies\\python\\python.exe _extract_docs.py
"""
import importlib.util
import os
import shutil
import subprocess
import sys

OUT_DIR = r"C:\aWorkspace\project-ra8p1\SLAM_Smart_Car\Renesas\TinySLAM\3.LSLidar\_docs"

PDF_JOBS = [
    (r"C:\aFile\百度网盘\N10系列激光雷达附送资料\N10系列雷达客户资料V5.0_20240822\4.产品手册\N10\N10数据输出协议V1.1.pdf",
     "n10_protocol.txt"),
    (r"C:\aFile\百度网盘\N10系列激光雷达附送资料\N10系列雷达客户资料V5.0_20240822\4.产品手册\N10\N10_规格书_v1.3.4.pdf",
     "n10_spec.txt"),
]

DOCX_JOBS = [
    (r"C:\aFile\百度网盘\2-竞赛\6.杭州电子科技大学圣光机学院比赛\报名\1.杭州电子科技大学圣光机联合学院第五届大学生智能机器人竞赛.docx",
     "competition.txt"),
]

PROBE_ORDER = ["pypdf", "PyPDF2", "fitz", "pdfplumber", "pdfminer"]


def have(mod):
    try:
        return importlib.util.find_spec(mod) is not None
    except (ImportError, ValueError):
        return False


def find_pdftotext():
    exe = shutil.which("pdftotext")
    if exe:
        return exe
    cands = [
        r"C:\Program Files\poppler\Library\bin\pdftotext.exe",
        r"C:\Program Files\poppler\bin\pdftotext.exe",
        r"C:\Program Files (x86)\poppler\bin\pdftotext.exe",
        r"C:\poppler\bin\pdftotext.exe",
        r"C:\ProgramData\chocolatey\bin\pdftotext.exe",
        r"C:\msys64\mingw64\bin\pdftotext.exe",
    ]
    for c in cands:
        if os.path.isfile(c):
            return c
    for root in (r"C:\Program Files", r"C:\Program Files (x86)", r"C:\tools", r"C:\dev"):
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            if dirpath.count(os.sep) - root.count(os.sep) > 4:
                dirnames[:] = []
                continue
            if "pdftotext.exe" in filenames:
                return os.path.join(dirpath, "pdftotext.exe")
    return None


def pages_with_pypdf(path, modname):
    mod = __import__(modname)
    reader = mod.PdfReader(path)
    return [(p.extract_text() or "") for p in reader.pages]


def pages_with_fitz(path):
    import fitz
    doc = fitz.open(path)
    try:
        return [page.get_text() for page in doc]
    finally:
        doc.close()


def pages_with_pdfplumber(path):
    import pdfplumber
    out = []
    with pdfplumber.open(path) as pdf:
        for page in pdf.pages:
            out.append(page.extract_text() or "")
    return out


def pages_with_pdfminer(path):
    from pdfminer.high_level import extract_pages
    from pdfminer.layout import LTTextContainer
    out = []
    for layout in extract_pages(path):
        buf = []
        for element in layout:
            if isinstance(element, LTTextContainer):
                buf.append(element.get_text())
        out.append("".join(buf))
    return out


def pages_with_pdftotext(path, exe, tmpdir):
    base = os.path.join(tmpdir, "pdftotext_out")
    subprocess.run([exe, "-enc", "UTF-8", "-layout", path, base], check=True,
                   capture_output=True)
    with open(base, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read()
    return raw.split("\f")


def extract_pdf(path, tmpdir):
    """Return (pages, method_name)."""
    errors = []
    if have("pypdf"):
        try:
            return pages_with_pypdf(path, "pypdf"), "pypdf"
        except Exception as exc:  # noqa: BLE001
            errors.append("pypdf: %r" % (exc,))
    if have("PyPDF2"):
        try:
            return pages_with_pypdf(path, "PyPDF2"), "PyPDF2"
        except Exception as exc:  # noqa: BLE001
            errors.append("PyPDF2: %r" % (exc,))
    if have("fitz"):
        try:
            return pages_with_fitz(path), "PyMuPDF (fitz)"
        except Exception as exc:  # noqa: BLE001
            errors.append("fitz: %r" % (exc,))
    if have("pdfplumber"):
        try:
            return pages_with_pdfplumber(path), "pdfplumber"
        except Exception as exc:  # noqa: BLE001
            errors.append("pdfplumber: %r" % (exc,))
    if have("pdfminer"):
        try:
            return pages_with_pdfminer(path), "pdfminer.six"
        except Exception as exc:  # noqa: BLE001
            errors.append("pdfminer: %r" % (exc,))
    exe = find_pdftotext()
    if exe:
        try:
            return pages_with_pdftotext(path, exe, tmpdir), "pdftotext (%s)" % exe
        except Exception as exc:  # noqa: BLE001
            errors.append("pdftotext: %r" % (exc,))
    else:
        errors.append("pdftotext.exe: not found on PATH or in common Poppler dirs")
    raise RuntimeError("no PDF extractor worked for %s\n  %s" % (path, "\n  ".join(errors)))


def write_pdf(path, out_name):
    tmpdir = os.path.join(OUT_DIR, "_tmp")
    os.makedirs(tmpdir, exist_ok=True)
    pages, method = extract_pdf(path, tmpdir)
    total = len(pages)
    chunks = []
    for idx, text in enumerate(pages, 1):
        text = text.replace("\r\n", "\n").replace("\r", "\n").strip("\n")
        chunks.append("===== PAGE %d / %d =====\n%s\n" % (idx, total, text))
    body = "\n".join(chunks)
    out_path = os.path.join(OUT_DIR, out_name)
    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(body)
    return out_path, method, total


def write_docx(path, out_name):
    import docx
    document = docx.Document(path)
    parts = []
    for para in document.paragraphs:
        parts.append(para.text)
    if document.tables:
        parts.append("")
        for t_idx, table in enumerate(document.tables, 1):
            parts.append("===== TABLE %d =====" % t_idx)
            for row in table.rows:
                cells = [c.text.replace("\n", " ").strip() for c in row.cells]
                parts.append(" | ".join(cells))
            parts.append("")
    body = "\n".join(parts).replace("\r\n", "\n")
    out_path = os.path.join(OUT_DIR, out_name)
    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(body)
    return out_path, "python-docx", None


def report(out_path, method, page_count):
    size = os.path.getsize(out_path)
    with open(out_path, "r", encoding="utf-8", errors="replace") as fh:
        head = fh.read(400)
    print("\n--- %s ---" % os.path.basename(out_path))
    print("path      : %s" % out_path)
    print("method    : %s" % method)
    print("bytes     : %d" % size)
    if page_count is not None:
        print("pages     : %d" % page_count)
    print("first 400 : %r" % head)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("importable PDF libs:", {m: have(m) for m in PROBE_ORDER})
    print("pdftotext:", find_pdftotext())

    failures = []
    for path, out_name in PDF_JOBS:
        if not os.path.isfile(path):
            failures.append("MISSING INPUT: %s" % path)
            continue
        try:
            out_path, method, pages = write_pdf(path, out_name)
            report(out_path, method, pages)
        except Exception as exc:  # noqa: BLE001
            failures.append("%s -> %r" % (path, exc))

    for path, out_name in DOCX_JOBS:
        if not os.path.isfile(path):
            failures.append("MISSING INPUT: %s" % path)
            continue
        try:
            out_path, method, _ = write_docx(path, out_name)
            report(out_path, method, None)
        except Exception as exc:  # noqa: BLE001
            failures.append("%s -> %r" % (path, exc))

    tmpdir = os.path.join(OUT_DIR, "_tmp")
    if os.path.isdir(tmpdir) and not os.listdir(tmpdir):
        os.rmdir(tmpdir)

    if failures:
        print("\nFAILURES:")
        for f in failures:
            print("  " + f)
        return 1
    print("\nAll requested files written.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
