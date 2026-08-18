#include <iostream>

#include "oscillator.h"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <terrain_file>\n";
        return 1;
    }

    td::Oscillator<double> osc{argv[1]};
    osc.GetNextSample();
}
