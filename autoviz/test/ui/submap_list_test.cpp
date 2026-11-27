// Copyright 2026 The Openbot Authors
#include <cstdio>
#include <functional>
#include <unistd.h>
#include <QApplication>
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/plugins/displays/submaps/submaps_display.hpp"
#include "autoviz/ui/displays/panel.hpp"

namespace proto = autonomy::localization::cartographer::proto;
namespace {
int failures = 0;
void Check(bool ok, const char* message) {
  if (!ok) { ++failures; std::fprintf(stderr, "FAIL: %s\n", message); }
}
QTreeWidgetItem* Find(QTreeWidgetItem* parent, const QString& key) {
  if (parent->data(0, autoviz::kDisplayTreeRolePropertyKey).toString() == key) return parent;
  for (int i=0;i<parent->childCount();++i) if (auto* found=Find(parent->child(i),key)) return found;
  return nullptr;
}
bool Wait(QApplication& app, const std::function<bool()>& ready) {
  for (int n=0;n<200;++n) {
    app.processEvents();
    if (ready()) return true;
    usleep(10000);
  }
  return false;
}
}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  autoviz::integration::AutolinkContext link;
  const auto suffix=std::to_string(getpid());
  if (!link.initialize(argv[0],"autoviz_submap_list_test_"+suffix)) return 1;
  auto& readers=autoviz::integration::ChannelReaderRegistry::instance();
  readers.setNode(link.node());
  const std::string channel="/codex_submap_list_test_"+suffix;
  auto writer=link.node()->CreateWriter<proto::SubmapList>(channel);
  autoviz::common::DisplayContext context;
  context.autolink=&link; context.fixed_frame="map";
  {
    auto manager=std::make_shared<autoviz::common::VisualizationManager>();
    auto config=autoviz::common::DisplayFactory::defaultForType("Submaps");
    config.channel=channel; config.enabled=false;
    Check(manager->addDisplay(config),"submap display is added to the actual panel manager");
    auto* display=manager->displayAt(0);
    if (!display) return 1;
    display->setContext(&context); display->setEnabled(true);
    display->setPropertyValue("show_textures","false");
    autoviz::DisplaysPanel panel(manager);
    panel.setAttribute(Qt::WA_DontShowOnScreen); panel.resize(560,760); panel.show();
    panel.armTreePainting();
    auto* tree=panel.findChild<autoviz::DisplayTreeWidget*>();
    if (!tree) return 1;
    QTreeWidgetItem* display_row=nullptr;
    for (int i=0;i<tree->topLevelItemCount();++i) {
      auto* row=tree->topLevelItem(i);
      if (row->data(0,autoviz::kDisplayTreeRoleKind).toInt()==static_cast<int>(autoviz::DisplayTreeItemKind::kDisplay)) display_row=row;
    }
    if (!display_row) return 1;
    display_row->setExpanded(true);
    Check(Find(display_row,"tracking_frame") && Find(display_row,"fade_out_distance"),
          "height fading controls appear in the Submaps property panel");
    auto* root=Find(display_row,"submaps");
    Check(root && !(root->flags() & Qt::ItemIsEditable),"submap list is a read-only category");
    if (!root) return 1;
    root->setExpanded(true);
    proto::SubmapList list; list.mutable_header()->set_frame_id("map");
    for (const auto& id: {std::pair<int,int>{0,7},{0,8},{1,3}}) {
      auto* entry=list.add_submap(); entry->set_trajectory_id(id.first); entry->set_submap_index(id.second);
      entry->set_submap_version(1); entry->mutable_pose()->mutable_orientation()->set_w(1);
    }
    auto publish=[&] { writer->Write(list); usleep(1000); display->update(); };
    Check(Wait(app,[&] { publish(); return Find(root,"submaps/visible/0/7")!=nullptr; }),
          "live messages populate the list without rebuilding the panel");
    auto* first=Find(root,"submaps/visible/0/7");
    auto* trajectory=Find(root,"submaps/trajectory/0");
    auto* all=Find(root,"submaps/all_enabled");
    if (!first || !trajectory || !all) return 1;
    trajectory->setExpanded(true);
    Check(first->parent()==trajectory && first->text(0)=="7.1","submaps are grouped by trajectory and show versions");
    auto geometry=[&] { autoviz::rendering::SceneOverlay scene; display->draw(scene); return scene.lineVertices().size(); };
    Check(geometry()==18,"all three submaps initially render");
    all->setCheckState(1,Qt::Unchecked);
    app.processEvents();
    Check(geometry()==0,"All Enabled disables every submap");
    first->setCheckState(1,Qt::Checked);
    Check(geometry()==6,"checking one row after All Enabled shows just that submap");
    all->setCheckState(1,Qt::Checked);
    app.processEvents();
    Check(geometry()==18,"All Enabled restores every submap");
    trajectory->setCheckState(1,Qt::Unchecked);
    app.processEvents();
    Check(geometry()==6,"trajectory checkbox changes only its own submaps");
    Check(QMetaObject::invokeMethod(tree,"itemDoubleClicked",Qt::DirectConnection,
          Q_ARG(QTreeWidgetItem*,first),Q_ARG(int,0)),"double-click signal is delivered");
    Check(geometry()==6 && display->propertyValue("submaps/visible/0/7","")=="true" &&
          display->propertyValue("submaps/all_enabled","")=="false","double-click shows only the selected submap");
    app.processEvents();
    Find(root,"submaps/visible/0/8")->setCheckState(1,Qt::Checked);
    Check(geometry()==12,"another submap can be added to the selection");
    QMetaObject::invokeMethod(tree,"itemDoubleClicked",Qt::DirectConnection,Q_ARG(QTreeWidgetItem*,first),Q_ARG(int,0));
    Check(geometry()==6,"double-clicking the same submap can restore single selection repeatedly");
    list.mutable_submap(0)->set_submap_version(2);
    auto* extra=list.add_submap(); *extra=list.submap(1); extra->set_submap_index(9);
    Check(Wait(app,[&] { publish(); return Find(root,"submaps/visible/0/9") && first->text(0)=="7.2"; }),
          "new submaps and versions update while the panel is open");
    Check(Find(root,"submaps/visible/0/7")==first && trajectory->isExpanded(),
          "existing row identity and expansion survive live updates");
    Check(geometry()==6,"new submaps inherit hidden defaults during single-submap viewing");
    display->setPropertyValue("tracking_frame","imu_link");
    display->setPropertyValue("fade_out_distance","2.5");
    autoviz::common::DisplayConfig saved; display->saveToConfig(&saved);
    auto restored=autoviz::common::DisplayFactory::create(saved);
    Check(restored && restored->propertyValue("submaps/visible/0/7","")=="true" &&
          restored->propertyValue("submaps/all_enabled","")=="false","visibility selections survive config round-trip");
    Check(restored && restored->propertyValue("tracking_frame","")=="imu_link" &&
          restored->propertyValue("fade_out_distance","")=="2.5","height fading settings survive config round-trip");
    if (argc>1) panel.grab().save(QString::fromLocal8Bit(argv[1]));
    list.mutable_submap()->DeleteSubrange(0,1);
    Check(Wait(app,[&] { publish(); return !Find(root,"submaps/visible/0/7"); }),"trimmed submaps disappear from the list");
    Check(geometry()==0,"trimmed selection leaves no stale geometry");
    list.clear_submap();
    Check(Wait(app,[&] { publish(); return !Find(root,"submaps/trajectory/0") && !Find(root,"submaps/trajectory/1"); }),
          "empty snapshots remove old trajectory groups safely");
    for (int i=0;i<500;++i) {
      auto* entry=list.add_submap(); entry->set_trajectory_id(3); entry->set_submap_index(1000+i);
      entry->set_submap_version(1); entry->mutable_pose()->mutable_orientation()->set_w(1);
    }
    Check(Wait(app,[&] { publish(); return Find(root,"submaps/visible/3/1499"); }),"large lists populate all 500 rows");
    auto* stable=Find(root,"submaps/visible/3/1000");
    for (int n=0;n<60;++n) {
      list.mutable_submap(0)->set_submap_version(n+2); publish(); app.processEvents();
    }
    Check(Wait(app,[&] { publish(); return stable->text(0)=="1000.61"; }) && Find(root,"submaps/visible/3/1000")==stable,
          "rapid live updates preserve row identity in a 500-submap list");
    list.clear_submap();
    Check(Wait(app,[&] { publish(); return !Find(root,"submaps/trajectory/3"); }),"large trajectory removal leaves no stale rows");
    display->setEnabled(false);
  }
  readers.setNode(nullptr);
  if (!failures) std::puts("PASS live Submaps tree, checkboxes, single selection, persistence and trimming");
  return failures ? 1 : 0;
}
