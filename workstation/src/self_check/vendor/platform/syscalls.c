/* No host filesystem, semihosting or libc heap in this firmware.
 * UART diagnostics use HAL_UART_Transmit explicitly. FreeRTOS heap_4 has its
 * own fixed array and does not use _sbrk. Fail accidental libc I/O/allocation.
 */
#include "main.h"
#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>

int _close(int file) { (void)file; errno = EBADF; return -1; }
int _lseek(int file, int offset, int direction)
{
    (void)file; (void)offset; (void)direction; errno = ESPIPE; return -1;
}
int _read(int file, char *buffer, int count)
{
    (void)file; (void)buffer; (void)count; errno = ENOSYS; return -1;
}
int _write(int file, const char *buffer, int count)
{
    (void)file; (void)buffer; (void)count; errno = ENOSYS; return -1;
}
int _fstat(int file, struct stat *status)
{
    (void)file; (void)status; errno = EBADF; return -1;
}
int _isatty(int file) { (void)file; errno = ENOTTY; return 0; }
void *_sbrk(ptrdiff_t increment)
{
    (void)increment; errno = ENOMEM; return (void *)-1;
}
int _getpid(void) { return 1; }
int _kill(int process, int signal)
{
    (void)process; (void)signal; errno = EINVAL; return -1;
}
void _exit(int status)
{
    (void)status;
    Error_Handler();
    for (;;) { }
}
