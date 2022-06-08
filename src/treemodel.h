#pragma once

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <QVector>

namespace rosbag_panel
{
class TreeItem
{
  public:
    explicit TreeItem(const QVector<QVariant>& data,
                      TreeItem* parentItem = nullptr,
                      Qt::CheckState checked = Qt::CheckState::Unchecked);
    ~TreeItem();

    void appendChild(TreeItem* child);

    TreeItem* child(int row) const;
    int childCount() const;
    int columnCount() const;
    QVariant data(int column) const;
    int row() const;
    TreeItem* parentItem();
    Qt::CheckState checked() const;
    void setChecked(Qt::CheckState state);

  private:
    QVector<TreeItem*> child_items_;
    QVector<QVariant> item_data_;
    TreeItem* parent_item_;
    Qt::CheckState checked_;
};

class TreeModel : public QAbstractItemModel
{
    Q_OBJECT

  public:
    explicit TreeModel(QList<QStringList>& topic_infos, QObject* parent = nullptr, bool flat = false);
    ~TreeModel();

    QVariant data(const QModelIndex& index, int role) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    void setCheckState(Qt::CheckState check_state);
    QStringList checkedTopics() const;
    void setCheckedTopics(const QStringList& topics);
    void setSingleCheckMode(bool single_check);

  private:
    void setupModelData(QList<QStringList>& topic_infos, TreeItem* parent, bool flat = false);
    void recursivelySetCheckState(TreeItem* item, Qt::CheckState check_state);
    void recursivelyUpdateCheckState(TreeItem* item);
    void recursivelyUpdateParentsCheckState(TreeItem* item);
    void evaluateCheckStateBasedOnChildren(TreeItem* item);
    void setCheckState(TreeItem* item, Qt::CheckState check_state);
    void recursivelyCollectCheckedTopics(const TreeItem* item, QStringList& topics) const;
    void recursivelySetCheckedTopics(TreeItem* item, const QStringList& topics);

    TreeItem* root_item_;
    bool single_check_{false};
};
} // namespace rosbag_panel