#include <iostream>
#include <unistd.h>

#define PROJECT_NAME "pld-managerd"

int main(int argc, char **argv) {
    if(argc != 1) {
        std::cout << argv[0] <<  "takes no arguments.\n";
        return 1;
    }
    std::cout << "This is project " << PROJECT_NAME << ".\n";
    while(1) {
        sleep(1);
    }
    return 0;
}
