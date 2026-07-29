#include "orbitlab/Persistence.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

extern "C" int LLVMFuzzerTestOneInput(
    const std::uint8_t* data,
    const std::size_t size) {
    if (size > 2'000'000) {
        return 0;
    }
    try {
        orbitlab::Simulation simulation;
        orbitlab::deserializeSimulation(
            simulation,
            std::string{reinterpret_cast<const char*>(data), size});
        static_cast<void>(orbitlab::serializeSimulation(simulation));
    } catch (...) {
        // Rejection is expected. Sanitizers report memory and undefined-behavior defects.
    }
    return 0;
}
