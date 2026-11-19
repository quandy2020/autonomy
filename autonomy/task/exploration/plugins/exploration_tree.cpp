/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/plugins/exploration_tree.hpp"

#include <stdexcept>

namespace autonomy::task::exploration {
namespace {
// Update is ticked during navigation, recovery, pause and saving. An optional
// failed return still permits saving a partial map; its failure stays recorded.
constexpr const char* kExplorationTree = R"xml(
<root BTCPP_format="4" main_tree_to_execute="Explore">
  <BehaviorTree ID="Explore">
    <ReactiveSequence>
      <UpdateExploration/>
      <Fallback>
        <Sequence>
          <ScriptCondition code="exploration_finishing"/>
          <Fallback><ReturnToStart/><AlwaysSuccess/></Fallback>
          <SaveExplorationMap/>
        </Sequence>
        <Sequence>
          <SensorReady/>
          <StartupObservation/>
          <Fallback>
            <CheckExplorationCompletion/>
            <Sequence>
              <SelectObservationPose/>
              <AdaptObservationPose/>
              <ObserveAtPose/>
            </Sequence>
          </Fallback>
        </Sequence>
      </Fallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>)xml";
}  // namespace

class ExplorationTree::Impl {
public:
    explicit Impl(std::shared_ptr<plugins::exploration::ExplorationBtContext> binding)
        : context(std::move(binding)) {
        if (!context || !context->session || !context->clock ||
            context->session->driver() != ExplorationDriver::kBehaviorTree)
            throw std::invalid_argument("native exploration tree requires a BT-driven session and clock");
        auto board = BT::Blackboard::create();
        board->set("exploration_context", context);
        board->set("exploration_finishing", false);
        RegisterBuiltinBtNodes(factory);
        tree = factory.createTreeFromText(kExplorationTree, board);
    }
    std::shared_ptr<plugins::exploration::ExplorationBtContext> context;
    BT::BehaviorTreeFactory factory;
    BT::Tree tree;
};

ExplorationTree::ExplorationTree(std::shared_ptr<plugins::exploration::ExplorationBtContext> context)
    : impl_(std::make_unique<Impl>(std::move(context))) {}
ExplorationTree::~ExplorationTree() { try { Halt(); } catch (...) {} }
ExplorationSnapshot ExplorationTree::Tick() {
    if (impl_->context->session->Snapshot().state != ExplorationState::kFinished)
        impl_->tree.tickOnce();
    return impl_->context->session->Snapshot();
}
void ExplorationTree::Halt() { impl_->tree.haltTree(); }
BT::Blackboard::Ptr ExplorationTree::blackboard() const { return impl_->tree.rootBlackboard(); }
}  // namespace autonomy::task::exploration
