#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

int main(int argc, char* argv[]) {
    int fd = shm_open("pure_uav_shm", O_CREAT | O_RDWR, 0666);
    ftruncate(fd, 0x400000);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }

    char* p = static_cast<char*>(mmap(NULL, 0x400000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    memset(p, 'A', 0x400000);
    munmap(p, 0x400000);
    close(fd);

    return 0;
}