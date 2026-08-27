/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_xml_io.hpp"

#include <algorithm>

#include <QFile>
#include <QHash>
#include <QRectF>
#include <QSet>
#include <QSizeF>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

namespace autoviz {
namespace behavior_tree {
namespace {

const QSet<QString> kReservedAttributes = {
    QStringLiteral("ID"),
    QStringLiteral("name"),
    QStringLiteral("_autoviz_x"),
    QStringLiteral("_autoviz_y"),
    QStringLiteral("_autoviz_description"),
    QStringLiteral("_description"),
    QStringLiteral("_skipIf"),
    QStringLiteral("_successIf"),
    QStringLiteral("_failureIf"),
    QStringLiteral("_while"),
    QStringLiteral("_onSuccess"),
    QStringLiteral("_onFailure"),
    QStringLiteral("_onHalted"),
    QStringLiteral("_post"),
};

BtNodeKind ResolveKind(const QString& registration_id, const QString& xml_tag,
                       const QHash<QString, BtNodeModel>& models) {
  if (registration_id == QLatin1String("SubTree")) {
    return BtNodeKind::kSubTree;
  }
  const BtNodeKind tag_kind = BtNodeKindFromTag(xml_tag);
  if (tag_kind != BtNodeKind::kUndefined) {
    return tag_kind;
  }
  if (models.contains(registration_id)) {
    return models.value(registration_id).kind;
  }
  const QHash<QString, BtNodeModel> builtins = BuiltinNodeModels();
  if (builtins.contains(registration_id)) {
    return builtins.value(registration_id).kind;
  }
  return BtNodeKind::kAction;
}

BtPortDirection PortDirectionFromTag(const QString& tag) {
  if (tag == QLatin1String("input_port")) {
    return BtPortDirection::kInput;
  }
  if (tag == QLatin1String("output_port")) {
    return BtPortDirection::kOutput;
  }
  if (tag == QLatin1String("inout_port")) {
    return BtPortDirection::kInOut;
  }
  return BtPortDirection::kInput;
}

void ParseTreeNodesModel(QXmlStreamReader& reader, QHash<QString, BtNodeModel>& models) {
  while (reader.readNextStartElement()) {
    const QString element = reader.name().toString();
    const BtNodeKind kind = BtNodeKindFromTag(element);
    if (kind == BtNodeKind::kUndefined || kind == BtNodeKind::kRoot ||
        kind == BtNodeKind::kSubTree) {
      reader.skipCurrentElement();
      continue;
    }

    const QXmlStreamAttributes attrs = reader.attributes();
    const QString registration_id = attrs.value(QLatin1String("ID")).toString();
    if (registration_id.isEmpty()) {
      reader.skipCurrentElement();
      continue;
    }

    BtNodeModel model;
    model.kind = kind;
    model.registration_id = registration_id;

    while (reader.readNextStartElement()) {
      const QString port_tag = reader.name().toString();
      const BtPortDirection direction = PortDirectionFromTag(port_tag);
      const QXmlStreamAttributes port_attrs = reader.attributes();
      BtPortModel port;
      port.name = port_attrs.value(QLatin1String("name")).toString();
      port.type_name = port_attrs.value(QLatin1String("type")).toString();
      port.default_value = port_attrs.value(QLatin1String("default")).toString();
      port.description = reader.readElementText().trimmed();
      port.direction = direction;
      model.ports.push_back(port);
    }

    models.insert(registration_id, model);
  }
}

int ParseBehaviorTreeNode(QXmlStreamReader& reader, BtAbsTree& tree,
                          const QHash<QString, BtNodeModel>& models, int parent_uid) {
  const QString xml_tag = reader.name().toString();
  const QXmlStreamAttributes attrs = reader.attributes();

  QString registration_id = xml_tag;
  if (xml_tag == QLatin1String("Action") || xml_tag == QLatin1String("Condition") ||
      xml_tag == QLatin1String("Control") || xml_tag == QLatin1String("Decorator")) {
    registration_id = attrs.value(QLatin1String("ID")).toString();
  }

  BtAbsNode node;
  node.uid = NextUid(tree);
  node.registration_id = registration_id;
  node.instance_name = attrs.value(QLatin1String("name")).toString();
  node.kind = ResolveKind(registration_id, xml_tag, models);

  if (attrs.hasAttribute(QStringLiteral("_autoviz_x")) &&
      attrs.hasAttribute(QStringLiteral("_autoviz_y"))) {
    node.pos.setX(attrs.value(QStringLiteral("_autoviz_x")).toDouble());
    node.pos.setY(attrs.value(QStringLiteral("_autoviz_y")).toDouble());
  }

  auto take_attr = [&](const QString& key, QString* out) {
    if (attrs.hasAttribute(key)) {
      *out = attrs.value(key).toString();
    }
  };
  take_attr(QStringLiteral("_skipIf"), &node.skip_if);
  take_attr(QStringLiteral("_successIf"), &node.success_if);
  take_attr(QStringLiteral("_failureIf"), &node.failure_if);
  take_attr(QStringLiteral("_while"), &node.while_script);
  take_attr(QStringLiteral("_onSuccess"), &node.on_success);
  take_attr(QStringLiteral("_onFailure"), &node.on_failure);
  take_attr(QStringLiteral("_onHalted"), &node.on_halted);
  take_attr(QStringLiteral("_post"), &node.post_script);
  if (attrs.hasAttribute(QStringLiteral("_autoviz_description"))) {
    node.description = attrs.value(QStringLiteral("_autoviz_description")).toString();
  } else if (attrs.hasAttribute(QStringLiteral("_description"))) {
    node.description = attrs.value(QStringLiteral("_description")).toString();
  }

  for (const QXmlStreamAttribute& attribute : attrs) {
    const QString key = attribute.name().toString();
    if (kReservedAttributes.contains(key)) {
      continue;
    }
    node.port_remap.insert(key, attribute.value().toString());
  }

  tree.nodes.insert(node.uid, node);
  if (parent_uid >= 0) {
    tree.nodes[parent_uid].children.push_back(node.uid);
  }

  while (reader.readNextStartElement()) {
    ParseBehaviorTreeNode(reader, tree, models, node.uid);
  }
  return node.uid;
}

void ParseBehaviorTree(QXmlStreamReader& reader, BtDocument& doc) {
  const QXmlStreamAttributes attrs = reader.attributes();
  BtAbsTree tree;
  tree.tree_id = attrs.value(QLatin1String("ID")).toString();

  // Groot2 always shows a visual Root node above the BehaviorTree entry.
  // It is editor-only and is not written back as a BT.CPP XML element.
  BtAbsNode root;
  root.uid = NextUid(tree);
  root.registration_id = QStringLiteral("Root");
  root.kind = BtNodeKind::kRoot;
  root.instance_name = QStringLiteral("RootTree");
  tree.nodes.insert(root.uid, root);
  tree.root_uid = root.uid;

  while (reader.readNextStartElement()) {
    const QString tag = reader.name().toString();
    if (tag.compare(QLatin1String("Root"), Qt::CaseInsensitive) == 0) {
      // Nested <Root> from older files: adopt its children under our visual Root.
      while (reader.readNextStartElement()) {
        ParseBehaviorTreeNode(reader, tree, doc.models, root.uid);
      }
    } else {
      ParseBehaviorTreeNode(reader, tree, doc.models, root.uid);
    }
  }

  if (tree.root_uid >= 0) {
    doc.trees.insert(tree.tree_id, tree);
  }
}

bool TreeHasLayout(const BtAbsTree& tree) {
  for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
    if (!it.value().pos.isNull()) {
      return true;
    }
  }
  return false;
}

// Approximate node box used only for tidy-tree packing. Visual sizes vary; spacing
// stays readable as long as sibling subtrees never overlap.
constexpr double kLayoutNodeW = 200.0;
constexpr double kLayoutNodeH = 90.0;
constexpr double kLayoutSiblingGap = 48.0;
constexpr double kLayoutLevelGapV = 72.0;
constexpr double kLayoutLevelGapH = 96.0;

QSizeF NodeLayoutSize(int uid, const QHash<int, QSizeF>& sizes) {
  return sizes.value(uid, QSizeF(kLayoutNodeW, kLayoutNodeH));
}

void TranslateSubtree(BtAbsTree& tree, int uid, const QPointF& delta, QSet<int>& visited) {
  if (!tree.nodes.contains(uid) || visited.contains(uid)) {
    return;
  }
  visited.insert(uid);
  tree.nodes[uid].pos += delta;
  for (int child_uid : tree.nodes.value(uid).children) {
    TranslateSubtree(tree, child_uid, delta, visited);
  }
}

QRectF SubtreeBounds(const BtAbsTree& tree, int uid, const QHash<int, QSizeF>& sizes,
                     QSet<int>& visited) {
  if (!tree.nodes.contains(uid) || visited.contains(uid)) {
    return {};
  }
  visited.insert(uid);
  const BtAbsNode& node = tree.nodes.value(uid);
  const QSizeF size = NodeLayoutSize(uid, sizes);
  QRectF bounds(node.pos.x(), node.pos.y(), size.width(), size.height());
  for (int child_uid : node.children) {
    const QRectF child_bounds = SubtreeBounds(tree, child_uid, sizes, visited);
    if (child_bounds.isValid()) {
      bounds = bounds.united(child_bounds);
    }
  }
  return bounds;
}

void NormalizeTreeOrigin(BtAbsTree& tree, const QHash<int, QSizeF>& sizes) {
  if (tree.root_uid < 0 || !tree.nodes.contains(tree.root_uid)) {
    return;
  }
  QSet<int> visited;
  const QRectF bounds = SubtreeBounds(tree, tree.root_uid, sizes, visited);
  if (!bounds.isValid()) {
    return;
  }
  QSet<int> shift_visited;
  TranslateSubtree(tree, tree.root_uid, QPointF(-bounds.left(), -bounds.top()), shift_visited);
}

/**
 * Top-down tidy tree: parent centered above children; sibling subtrees packed
 * left-to-right without overlap. Returns subtree width.
 */
double LayoutSubtreeVertical(BtAbsTree& tree, int uid, double y,
                             const QHash<int, QSizeF>& sizes, QSet<int>& visited) {
  if (!tree.nodes.contains(uid) || visited.contains(uid)) {
    return NodeLayoutSize(uid, sizes).width();
  }
  visited.insert(uid);

  const QSizeF self = NodeLayoutSize(uid, sizes);
  const QVector<int> children = tree.nodes.value(uid).children;
  if (children.isEmpty()) {
    tree.nodes[uid].pos = QPointF(0.0, y);
    return self.width();
  }

  const double child_y = y + self.height() + kLayoutLevelGapV;
  QVector<double> child_widths;
  child_widths.reserve(children.size());
  for (int child_uid : children) {
    child_widths.push_back(LayoutSubtreeVertical(tree, child_uid, child_y, sizes, visited));
  }

  double cursor = 0.0;
  for (int i = 0; i < children.size(); ++i) {
    const int child_uid = children.at(i);
    QSet<int> bound_visited;
    const QRectF bounds = SubtreeBounds(tree, child_uid, sizes, bound_visited);
    const double dx = cursor - (bounds.isValid() ? bounds.left() : 0.0);
    QSet<int> shift_visited;
    TranslateSubtree(tree, child_uid, QPointF(dx, 0.0), shift_visited);
    cursor += child_widths.at(i) + kLayoutSiblingGap;
  }

  const QSizeF first_size = NodeLayoutSize(children.first(), sizes);
  const QSizeF last_size = NodeLayoutSize(children.last(), sizes);
  const double first_center = tree.nodes.value(children.first()).pos.x() + first_size.width() * 0.5;
  const double last_center = tree.nodes.value(children.last()).pos.x() + last_size.width() * 0.5;
  tree.nodes[uid].pos = QPointF((first_center + last_center) * 0.5 - self.width() * 0.5, y);

  QSet<int> span_visited;
  const QRectF span = SubtreeBounds(tree, uid, sizes, span_visited);
  return span.isValid() ? span.width() : self.width();
}

/**
 * Left-to-right tidy tree: parent centered left of children; sibling subtrees
 * packed top-to-bottom without overlap. Returns subtree height.
 */
double LayoutSubtreeHorizontal(BtAbsTree& tree, int uid, double x,
                               const QHash<int, QSizeF>& sizes, QSet<int>& visited) {
  if (!tree.nodes.contains(uid) || visited.contains(uid)) {
    return NodeLayoutSize(uid, sizes).height();
  }
  visited.insert(uid);

  const QSizeF self = NodeLayoutSize(uid, sizes);
  const QVector<int> children = tree.nodes.value(uid).children;
  if (children.isEmpty()) {
    tree.nodes[uid].pos = QPointF(x, 0.0);
    return self.height();
  }

  const double child_x = x + self.width() + kLayoutLevelGapH;
  QVector<double> child_heights;
  child_heights.reserve(children.size());
  for (int child_uid : children) {
    child_heights.push_back(LayoutSubtreeHorizontal(tree, child_uid, child_x, sizes, visited));
  }

  double cursor = 0.0;
  for (int i = 0; i < children.size(); ++i) {
    const int child_uid = children.at(i);
    QSet<int> bound_visited;
    const QRectF bounds = SubtreeBounds(tree, child_uid, sizes, bound_visited);
    const double dy = cursor - (bounds.isValid() ? bounds.top() : 0.0);
    QSet<int> shift_visited;
    TranslateSubtree(tree, child_uid, QPointF(0.0, dy), shift_visited);
    cursor += child_heights.at(i) + kLayoutSiblingGap;
  }

  const QSizeF first_size = NodeLayoutSize(children.first(), sizes);
  const QSizeF last_size = NodeLayoutSize(children.last(), sizes);
  const double first_center = tree.nodes.value(children.first()).pos.y() + first_size.height() * 0.5;
  const double last_center = tree.nodes.value(children.last()).pos.y() + last_size.height() * 0.5;
  tree.nodes[uid].pos = QPointF(x, (first_center + last_center) * 0.5 - self.height() * 0.5);

  QSet<int> span_visited;
  const QRectF span = SubtreeBounds(tree, uid, sizes, span_visited);
  return span.isValid() ? span.height() : self.height();
}

QString ElementTagForNode(const BtAbsNode& node) {
  if (node.kind == BtNodeKind::kSubTree ||
      node.registration_id == QLatin1String("SubTree")) {
    return QStringLiteral("SubTree");
  }
  return node.registration_id;
}

void WriteNodeAttributes(QXmlStreamWriter& writer, const BtAbsNode& node) {
  if (!node.instance_name.isEmpty()) {
    writer.writeAttribute(QStringLiteral("name"), node.instance_name);
  }

  auto write_if = [&](const QString& key, const QString& value) {
    if (!value.isEmpty()) {
      writer.writeAttribute(key, value);
    }
  };
  write_if(QStringLiteral("_skipIf"), node.skip_if);
  write_if(QStringLiteral("_successIf"), node.success_if);
  write_if(QStringLiteral("_failureIf"), node.failure_if);
  write_if(QStringLiteral("_while"), node.while_script);
  write_if(QStringLiteral("_onSuccess"), node.on_success);
  write_if(QStringLiteral("_onFailure"), node.on_failure);
  write_if(QStringLiteral("_onHalted"), node.on_halted);
  write_if(QStringLiteral("_post"), node.post_script);
  write_if(QStringLiteral("_autoviz_description"), node.description);

  for (auto it = node.port_remap.constBegin(); it != node.port_remap.constEnd(); ++it) {
    writer.writeAttribute(it.key(), it.value());
  }

  if (!node.pos.isNull()) {
    writer.writeAttribute(QStringLiteral("_autoviz_x"), QString::number(node.pos.x(), 'f', 1));
    writer.writeAttribute(QStringLiteral("_autoviz_y"), QString::number(node.pos.y(), 'f', 1));
  }
}

void WriteBehaviorTreeNode(QXmlStreamWriter& writer, const BtAbsTree& tree, int uid) {
  if (!tree.nodes.contains(uid)) {
    return;
  }

  const BtAbsNode& node = tree.nodes.value(uid);
  writer.writeStartElement(ElementTagForNode(node));
  WriteNodeAttributes(writer, node);

  for (int child_uid : node.children) {
    WriteBehaviorTreeNode(writer, tree, child_uid);
  }

  writer.writeEndElement();
}

QString PortTagForDirection(BtPortDirection direction) {
  switch (direction) {
    case BtPortDirection::kInput:
      return QStringLiteral("input_port");
    case BtPortDirection::kOutput:
      return QStringLiteral("output_port");
    case BtPortDirection::kInOut:
      return QStringLiteral("inout_port");
  }
  return QStringLiteral("input_port");
}

void WriteTreeNodesModel(QXmlStreamWriter& writer, const QHash<QString, BtNodeModel>& models) {
  const QHash<QString, BtNodeModel> builtins = BuiltinNodeModels();
  bool wrote_section = false;

  for (auto it = models.constBegin(); it != models.constEnd(); ++it) {
    if (builtins.contains(it.key())) {
      continue;
    }
    if (!wrote_section) {
      writer.writeStartElement(QStringLiteral("TreeNodesModel"));
      wrote_section = true;
    }

    const BtNodeModel& model = it.value();
    writer.writeStartElement(TagFromBtNodeKind(model.kind));
    writer.writeAttribute(QStringLiteral("ID"), model.registration_id);
    for (const BtPortModel& port : model.ports) {
      writer.writeStartElement(PortTagForDirection(port.direction));
      writer.writeAttribute(QStringLiteral("name"), port.name);
      if (!port.type_name.isEmpty()) {
        writer.writeAttribute(QStringLiteral("type"), port.type_name);
      }
      if (!port.default_value.isEmpty()) {
        writer.writeAttribute(QStringLiteral("default"), port.default_value);
      }
      if (!port.description.isEmpty()) {
        writer.writeCharacters(port.description);
      }
      writer.writeEndElement();
    }
    writer.writeEndElement();
  }

  if (wrote_section) {
    writer.writeEndElement();
  }
}

}  // namespace

void ApplyVerticalTreeLayout(BtAbsTree& tree, const QHash<int, QSizeF>& node_sizes) {
  if (tree.root_uid < 0) {
    return;
  }
  QSet<int> visited;
  LayoutSubtreeVertical(tree, tree.root_uid, 0.0, node_sizes, visited);
  NormalizeTreeOrigin(tree, node_sizes);
}

void ApplyHorizontalTreeLayout(BtAbsTree& tree, const QHash<int, QSizeF>& node_sizes) {
  if (tree.root_uid < 0) {
    return;
  }
  QSet<int> visited;
  LayoutSubtreeHorizontal(tree, tree.root_uid, 0.0, node_sizes, visited);
  NormalizeTreeOrigin(tree, node_sizes);
}

std::optional<QHash<QString, BtNodeModel>> LoadTreeNodesModelFile(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return std::nullopt;
  }

  QHash<QString, BtNodeModel> models = BuiltinNodeModels();
  QXmlStreamReader reader(&file);
  while (!reader.atEnd()) {
    reader.readNext();
    if (!reader.isStartElement()) {
      continue;
    }
    if (reader.name() == QLatin1String("TreeNodesModel") ||
        reader.name() == QLatin1String("root")) {
      if (reader.name() == QLatin1String("TreeNodesModel")) {
        ParseTreeNodesModel(reader, models);
      } else {
        while (reader.readNextStartElement()) {
          if (reader.name() == QLatin1String("TreeNodesModel")) {
            ParseTreeNodesModel(reader, models);
          } else {
            reader.skipCurrentElement();
          }
        }
      }
    }
  }
  if (reader.hasError()) {
    return std::nullopt;
  }
  return models;
}

std::optional<QHash<QString, BtNodeModel>> LoadImportedCustomModels(const QString& path) {
  const auto models = LoadTreeNodesModelFile(path);
  if (!models.has_value()) {
    return std::nullopt;
  }
  QHash<QString, BtNodeModel> custom_only;
  const QHash<QString, BtNodeModel> builtins = BuiltinNodeModels();
  for (auto it = models->constBegin(); it != models->constEnd(); ++it) {
    if (!builtins.contains(it.key())) {
      custom_only.insert(it.key(), it.value());
    }
  }
  return custom_only;
}

bool SaveTreeNodesModelFile(const QHash<QString, BtNodeModel>& models, const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QXmlStreamWriter writer(&file);
  writer.setAutoFormatting(true);
  writer.writeStartDocument();
  writer.writeStartElement(QStringLiteral("root"));
  writer.writeAttribute(QStringLiteral("BTCPP_format"), QStringLiteral("4"));
  WriteTreeNodesModel(writer, models);
  writer.writeEndElement();
  writer.writeEndDocument();
  return true;
}

std::optional<BtDocument> LoadBtDocument(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return std::nullopt;
  }

  BtDocument doc;
  doc.source_path = path;
  doc.models = BuiltinNodeModels();

  QXmlStreamReader reader(&file);
  while (!reader.atEnd()) {
    reader.readNext();
    if (!reader.isStartElement()) {
      continue;
    }

    const QString element = reader.name().toString();
    if (element == QLatin1String("root")) {
      doc.main_tree_id = reader.attributes().value(QLatin1String("main_tree_to_execute")).toString();
      while (reader.readNextStartElement()) {
        const QString child = reader.name().toString();
        if (child == QLatin1String("TreeNodesModel")) {
          ParseTreeNodesModel(reader, doc.models);
        } else if (child == QLatin1String("BehaviorTree")) {
          ParseBehaviorTree(reader, doc);
        } else {
          reader.skipCurrentElement();
        }
      }
    } else if (element == QLatin1String("BehaviorTree")) {
      ParseBehaviorTree(reader, doc);
    } else if (element == QLatin1String("TreeNodesModel")) {
      ParseTreeNodesModel(reader, doc.models);
    } else {
      reader.skipCurrentElement();
    }
  }

  if (reader.hasError()) {
    return std::nullopt;
  }

  for (auto it = doc.trees.begin(); it != doc.trees.end(); ++it) {
    if (!TreeHasLayout(it.value())) {
      ApplyVerticalTreeLayout(it.value());
    }
  }

  if (doc.main_tree_id.isEmpty() && !doc.trees.isEmpty()) {
    doc.main_tree_id = doc.trees.constBegin().key();
  }

  return doc;
}

bool SaveBtDocument(const BtDocument& doc, const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    return false;
  }

  QXmlStreamWriter writer(&file);
  writer.setAutoFormatting(true);
  writer.writeStartDocument();
  writer.writeStartElement(QStringLiteral("root"));
  writer.writeAttribute(QStringLiteral("BTCPP_format"), QStringLiteral("4"));
  if (!doc.main_tree_id.isEmpty()) {
    writer.writeAttribute(QStringLiteral("main_tree_to_execute"), doc.main_tree_id);
  }

  WriteTreeNodesModel(writer, doc.models);

  const QStringList tree_ids = [&doc]() {
    QStringList ids = doc.trees.keys();
    ids.sort();
    return ids;
  }();

  for (const QString& tree_id : tree_ids) {
    const BtAbsTree& tree = doc.trees.value(tree_id);
    writer.writeStartElement(QStringLiteral("BehaviorTree"));
    writer.writeAttribute(QStringLiteral("ID"), tree.tree_id);
    if (tree.root_uid >= 0 && tree.nodes.contains(tree.root_uid)) {
      const BtAbsNode& root = tree.nodes.value(tree.root_uid);
      if (root.kind == BtNodeKind::kRoot ||
          root.registration_id == QLatin1String("Root")) {
        // Visual Root is not part of BT.CPP XML — emit its children only.
        for (int child_uid : root.children) {
          WriteBehaviorTreeNode(writer, tree, child_uid);
        }
      } else {
        WriteBehaviorTreeNode(writer, tree, tree.root_uid);
      }
    }
    writer.writeEndElement();
  }

  writer.writeEndElement();
  writer.writeEndDocument();
  return true;
}

}  // namespace behavior_tree
}  // namespace autoviz
