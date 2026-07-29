#pragma once

#include "orbitlab/Simulation.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace orbitlab::app {

class History {
public:
    void reset(const Simulation& simulation);
    void commit(const Simulation& simulation);
    [[nodiscard]] bool canUndo() const noexcept;
    [[nodiscard]] bool canRedo() const noexcept;
    bool undo(Simulation& simulation);
    bool redo(Simulation& simulation);
    bool restoreInitial(Simulation& simulation);

private:
    static constexpr std::size_t maximumSnapshots = 96;
    std::vector<std::string> snapshots_;
    std::size_t cursor_{0};
};

} // namespace orbitlab::app
