// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_panel.hpp
 * @brief Region coverage controls embedded in the navigation panel. */
#pragma once
#ifdef AUTONOMY_BUILD_COVERAGE
#include <QWidget>
#include <future>
#include <memory>
#pragma push_macro("forever")
#undef forever
#include "autolink/action/client.hpp"
#pragma pop_macro("forever")
#include "autolink/node/writer.hpp"
#include "autonomy/task/navigation/coverage/proto/coverage.pb.h"
class QPlainTextEdit;
class QLabel;
class QLineEdit;
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QCheckBox;
class QTimer;
namespace autoviz::common {
class VisualizationManager;
}
namespace autoviz::navigation {
/** @brief Submit, preview and cancel region coverage with correlated native
 * actions. */
class CoveragePanel : public QWidget {
  Q_OBJECT
 public:
  /** @brief Bind controls to the visualization manager's native node. */
  explicit CoveragePanel(common::VisualizationManager*,
                         QWidget* parent = nullptr);
  ~CoveragePanel() override;
  /** @brief Cancel only goals owned by this coverage panel. */
  void CancelCoverage();
 private slots:
  /** @brief Compute a complete path and publish its preview. */
  void PreviewCoverage();
  /** @brief Submit the edited field as a complete coverage navigation task. */
  void StartCoverage();
  /** @brief Observe native acceptance, progress and terminal results. */
  void PollActions();

 private:
  using Compute = autonomy::task::coverage::proto::ComputeCoveragePathAction;
  using Navigate =
      autonomy::task::coverage::proto::NavigateCompleteCoverageAction;
  using ComputeClient = autolink::action::Client<Compute>;
  using NavigateClient = autolink::action::Client<Navigate>;
  /** @brief Validate edited region rings and construct the native request. */
  autonomy::task::coverage::proto::CoverageRequest BuildRequest() const;
  /** @brief Initialize native clients and the persistent full-path preview
   * display. */
  void EnsureClients();
  /** @brief Synchronize control availability with outstanding goal ownership.
   */
  void SetBusy(bool);
  common::VisualizationManager* manager_{nullptr};
  QPlainTextEdit* polygons_{nullptr};
  QLineEdit* frame_{nullptr};
  QDoubleSpinBox* headland_{nullptr};
  QComboBox* route_mode_{nullptr};
  QComboBox* path_mode_{nullptr};
  QCheckBox* approach_{nullptr};
  QPushButton* preview_{nullptr};
  QPushButton* start_{nullptr};
  QPushButton* cancel_{nullptr};
  QLabel* status_{nullptr};
  QTimer* timer_{nullptr};
  bool busy_{false}, cancel_requested_{false};
  std::shared_ptr<ComputeClient> compute_client_;
  std::shared_ptr<NavigateClient> navigate_client_;
  std::shared_future<
      std::shared_ptr<autolink::action::ClientGoalHandle<Compute>>>
      compute_acceptance_;
  std::shared_future<
      std::shared_ptr<autolink::action::ClientGoalHandle<Navigate>>>
      navigate_acceptance_;
  std::shared_ptr<autolink::action::ClientGoalHandle<Compute>> compute_goal_;
  std::shared_ptr<autolink::action::ClientGoalHandle<Navigate>> navigate_goal_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      preview_writer_;
};
}  // namespace autoviz::navigation
#endif
