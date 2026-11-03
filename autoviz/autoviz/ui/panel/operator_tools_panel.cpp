/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/operator_tools_panel.hpp"

#include <QTabWidget>
#include <QVBoxLayout>

#include "autoviz/ui/service/cancel_action_panel.hpp"
#include "autoviz/ui/service/empty_service_call_panel.hpp"
#include "autoviz/ui/object_fit/object_fit_operator_panel.hpp"
#include "autoviz/ui/publish/publish_topic_panel.hpp"
#include "autoviz/ui/record/record_action_panel.hpp"
#include "autoviz/ui/teleop/robot_command_interface_panel.hpp"
#include "autoviz/ui/publish/select_point_cloud_publish_panel.hpp"
#include "autoviz/ui/teleop/tablet_controller_panel.hpp"
#include "autoviz/ui/dialog/yes_no_button_panel.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

OperatorToolsPanel::OperatorToolsPanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : QWidget(parent) {
  ApplyPanelShell(this);
  auto* tabs = new QTabWidget(this);
  tabs->addTab(new YesNoButtonPanel(manager, tabs), tr("Yes/No"));
  tabs->addTab(new RecordActionPanel(manager, tabs), tr("Record"));
  tabs->addTab(new PublishTopicPanel(manager, tabs), tr("Publish"));
  tabs->addTab(new EmptyServiceCallPanel(manager, tabs), tr("Empty Call"));
  tabs->addTab(new ObjectFitOperatorPanel(manager, tabs), tr("Object Fit"));
  tabs->addTab(new RobotCommandInterfacePanel(manager, tabs), tr("Robot Cmd"));
  tabs->addTab(new TabletControllerPanel(manager, tabs), tr("Tablet"));
  tabs->addTab(new CancelActionPanel(manager, tabs), tr("Cancel"));
  tabs->addTab(new SelectPointCloudPublishPanel(manager, tabs),
               tr("Select Cloud"));
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(tabs);
}

}  // namespace autoviz
