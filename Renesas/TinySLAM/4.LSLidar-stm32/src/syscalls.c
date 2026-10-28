#include <sys/stat.h>      // struct stat / S_IFCHR
#include <errno.h>         // ENOSYS

/* 下面 5 个是 newlib 的系统调用钩子。裸机没有文件系统，
 * 程序也不会真的调用它们，所以只需占位返回错误即可。 */

int _close(int fd)
{
    (void)fd;
    return -1;             // 关闭文件：不支持
}

int _lseek(int fd, int ptr, int dir)
{
    (void)fd; (void)ptr; (void)dir;
    return -1;             // 文件定位：不支持
}

int _read(int fd, char *ptr, int len)
{
    (void)fd; (void)ptr; (void)len;
    return -1;             // 读文件：不支持（串口接收我们不用 newlib 的 read）
}

int _fstat(int fd, struct stat *st)
{
    (void)fd;
    st->st_mode = S_IFCHR; // 假装这是个"字符设备"（串口就是字符设备）
    return 0;
}

int _isatty(int fd)
{
    (void)fd;
    return 1;              // 假装是终端（tty），避免某些库走文件缓冲
}