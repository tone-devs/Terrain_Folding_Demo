#include <iostream>

#include "globals.hpp"
#include "oscillator.hpp"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <terrain_file>\n";
        return 1;
    }

    td::Oscillator<double> osc{argv[1]};
    std::array<std::array<float, td::kBlockSize>, 2> audio_block;
    osc.GetNextBlock(audio_block);
}
