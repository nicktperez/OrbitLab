#include "app/History.hpp"

#include "orbitlab/Persistence.hpp"

#include <algorithm>

namespace orbitlab::app {

void History::reset(const Simulation& simulation) {
    snapshots_.assign(1, serializeSimulation(simulation));
    cursor_ = 0;
}

void History::commit(const Simulation& simulation) {
    const std::string snapshot = serializeSimulation(simulation);
    if (!snapshots_.empty() && snapshots_[cursor_] == snapshot) {
        return;
    }
    if (!snapshots_.empty()) {
        snapshots_.erase(
            snapshots_.begin() + static_cast<std::ptrdiff_t>(cursor_ + 1),
            snapshots_.end());
    }
    snapshots_.push_back(snapshot);
    if (snapshots_.size() > maximumSnapshots) {
        snapshots_.erase(snapshots_.begin() + 1);
    }
    cursor_ = snapshots_.size() - 1;
}

bool History::canUndo() const noexcept {
    return !snapshots_.empty() && cursor_ > 0;
}

bool History::canRedo() const noexcept {
    return !snapshots_.empty() && cursor_ + 1 < snapshots_.size();
}

bool History::undo(Simulation& simulation) {
    if (!canUndo()) {
        return false;
    }
    deserializeSimulation(simulation, snapshots_[--cursor_]);
    return true;
}

bool History::redo(Simulation& simulation) {
    if (!canRedo()) {
        return false;
    }
    deserializeSimulation(simulation, snapshots_[++cursor_]);
    return true;
}

bool History::restoreInitial(Simulation& simulation) {
    if (snapshots_.empty()) {
        return false;
    }
    deserializeSimulation(simulation, snapshots_.front());
    commit(simulation);
    return true;
}

} // namespace orbitlab::app
