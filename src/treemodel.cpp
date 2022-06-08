#include <QFlags>
#include <QStringList>
#include <iostream>

#include "treemodel.h"

namespace rosbag_panel
{

TreeItem::TreeItem(const QVector<QVariant>& data, TreeItem* parent, Qt::CheckState checked)
    : item_data_(data), parent_item_(parent), checked_(checked)
{
}

TreeItem::~TreeItem()
{
    qDeleteAll(child_items_);
}

void TreeItem::appendChild(TreeItem* item)
{
    child_items_.append(item);
}

TreeItem* TreeItem::child(int row) const
{
    if (row < 0 || row >= child_items_.size())
        return nullptr;
    return child_items_.at(row);
}

int TreeItem::childCount() const
{
    return child_items_.count();
}

int TreeItem::columnCount() const
{
    return item_data_.count();
}

QVariant TreeItem::data(int column) const
{
    if (column < 0 || column >= item_data_.size())
        return QVariant();
    return item_data_.at(column);
}

TreeItem* TreeItem::parentItem()
{
    return parent_item_;
}

int TreeItem::row() const
{
    if (parent_item_)
        return parent_item_->child_items_.indexOf(const_cast<TreeItem*>(this));

    return 0;
}

Qt::CheckState TreeItem::checked() const
{
    return checked_;
}

void TreeItem::setChecked(Qt::CheckState state)
{
    checked_ = state;
}

TreeModel::TreeModel(QList<QStringList>& topic_infos, QObject* parent, bool flat) : QAbstractItemModel(parent)
{
    root_item_ = new TreeItem({tr("Topic"), tr("Message Type"), tr("Latched")});
    setupModelData(topic_infos, root_item_, flat);
}

TreeModel::~TreeModel()
{
    delete root_item_;
}

int TreeModel::columnCount(const QModelIndex& parent) const
{
    if (parent.isValid())
        return static_cast<TreeItem*>(parent.internalPointer())->columnCount();
    return root_item_->columnCount();
}

QVariant TreeModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    auto* item = static_cast<TreeItem*>(index.internalPointer());

    if (role == Qt::CheckStateRole && index.column() == 0)
        return item->checked();

    if (role != Qt::DisplayRole)
        return QVariant();

    return item->data(index.column());
}

bool TreeModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (!index.isValid())
    {
        return false;
    }

    if (role == Qt::CheckStateRole && index.column() == 0)
    {
        auto* item = static_cast<TreeItem*>(index.internalPointer());

        const auto check_state = static_cast<Qt::CheckState>(value.toInt());

        if (single_check_)
        {
            // make only leaves checkable
            if (item->childCount() == 0)
            {
                if (item->checked())
                {
                    setCheckState(item, Qt::Unchecked);
                }
                else
                {
                    recursivelySetCheckState(root_item_, Qt::Unchecked); // uncheck others
                    setCheckState(item, Qt::Checked);
                }
                return true;
            }
            return false;
        }

        recursivelySetCheckState(item, check_state);
        recursivelyUpdateParentsCheckState(item);
        return true;
    }

    return false;
}

Qt::ItemFlags TreeModel::flags(const QModelIndex& index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    if (index.column() == 0)
    {
        flags |= Qt::ItemIsUserCheckable;
        flags |= Qt::ItemIsTristate;
    }

    return flags;
}

QVariant TreeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return root_item_->data(section);

    return QVariant();
}

QModelIndex TreeModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    TreeItem* parentItem;

    if (!parent.isValid())
        parentItem = root_item_;
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    TreeItem* childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    return QModelIndex();
}

QModelIndex TreeModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return QModelIndex();

    TreeItem* childItem = static_cast<TreeItem*>(index.internalPointer());
    TreeItem* parentItem = childItem->parentItem();

    if (parentItem == root_item_)
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

int TreeModel::rowCount(const QModelIndex& parent) const
{
    TreeItem* parentItem;
    if (parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = root_item_;
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    return parentItem->childCount();
}

void TreeModel::setupModelData(QList<QStringList>& topic_infos, TreeItem* parent, bool flat)
{
    // sort by topic name
    std::sort(
        topic_infos.begin(), topic_infos.end(), [](const QStringList& a, const QStringList& b) { return a[0] < b[0]; });

    if (flat)
    {
        for (const auto& topic_info : topic_infos)
        {
            QVector<QVariant> column_data;
            column_data << topic_info[0];
            column_data << topic_info[1];
            column_data << topic_info[2];
            column_data << topic_info[0]; // add topic again to be consistent to non-flat version
            parent->appendChild(new TreeItem(column_data, parent));
        }
        return;
    }

    QVector<TreeItem*> parents;
    QStringList prev_split_topics;
    parents << parent;

    for (const QStringList& topic_info : topic_infos)
    {
        int position = 0;

        QStringList split_topics = topic_info[0].split(QLatin1Char('/'), QString::SkipEmptyParts);

        while (position < split_topics.size() && position < prev_split_topics.size() &&
               split_topics[position] == prev_split_topics[position])
        {
            position++;
        }

        parents = parents.mid(0, position + 1);

        for (; position < split_topics.size(); ++position)
        {
            QVector<QVariant> column_data;
            column_data << "/" + split_topics[position];
            column_data << (position == split_topics.size() - 1 ? topic_info[1] : "");
            column_data << (position == split_topics.size() - 1 ? topic_info[2] : "");
            column_data << (position == split_topics.size() - 1 ? topic_info[0] : ""); // not displayed (just store)

            parents.last()->appendChild(new TreeItem(column_data, parents.last()));
            parents << parents.last()->child(parents.last()->childCount() - 1);
        }

        prev_split_topics = split_topics;
    }
}

void TreeModel::recursivelySetCheckState(TreeItem* item, Qt::CheckState check_state)
{
    setCheckState(item, check_state);
    // do this recursively for all children
    for (int i = 0; i < item->childCount(); i++)
        recursivelySetCheckState(item->child(i), check_state);
}

void TreeModel::recursivelyUpdateParentsCheckState(TreeItem* item)
{
    auto parent = item->parentItem();
    while (parent != nullptr)
    {
        evaluateCheckStateBasedOnChildren(parent);
        parent = parent->parentItem();
    }
}

void TreeModel::recursivelyUpdateCheckState(TreeItem* item)
{
    // recursively update children
    for (int i = 0; i < item->childCount(); i++)
    {
        recursivelyUpdateCheckState(item->child(i));
    }
    // update current item
    evaluateCheckStateBasedOnChildren(item);
}

void TreeModel::evaluateCheckStateBasedOnChildren(TreeItem* item)
{
    if (item->childCount() == 0)
        return;

    // check if the check state of children are all the same, if not the current item will be partially checked
    Qt::CheckState state = item->child(0)->checked();
    for (int i = 1; i < item->childCount(); i++)
    {
        if (item->child(i)->checked() != state)
        {
            state = Qt::PartiallyChecked;
            break;
        }
    }

    // update check state of current item if necessary
    if (state != item->checked())
    {
        setCheckState(item, state);
    }
}

void TreeModel::setCheckState(TreeItem* item, Qt::CheckState check_state)
{
    item->setChecked(check_state);
    // notify view about this change
    if (item != root_item_)
    {
        const QModelIndex index = createIndex(item->row(), 0, item);
        Q_EMIT dataChanged(index, index);
    }
}

void TreeModel::setCheckState(Qt::CheckState check_state)
{
    recursivelySetCheckState(root_item_, check_state);
}

void TreeModel::recursivelyCollectCheckedTopics(const TreeItem* item, QStringList& topics) const
{
    if (!item->data(3).toString().isEmpty() && item->checked() == Qt::Checked)
        topics.push_back(item->data(3).toString());

    for (int i = 0; i < item->childCount(); i++)
        recursivelyCollectCheckedTopics(item->child(i), topics);
}

QStringList TreeModel::checkedTopics() const
{
    QStringList topics;
    recursivelyCollectCheckedTopics(root_item_, topics);
    return topics;
}

void TreeModel::recursivelySetCheckedTopics(TreeItem* item, const QStringList& topics)
{
    setCheckState(item, Qt::Unchecked);

    for (const QString& topic : topics)
    {
        if (item->data(3).toString() == topic)
        {
            setCheckState(item, Qt::Checked);
            break;
        }
    }

    for (int i = 0; i < item->childCount(); i++)
        recursivelySetCheckedTopics(item->child(i), topics);
}

void TreeModel::setCheckedTopics(const QStringList& topics)
{
    recursivelySetCheckedTopics(root_item_, topics);
    recursivelyUpdateCheckState(root_item_);
}

void TreeModel::setSingleCheckMode(bool single_check)
{
    single_check_ = single_check;
}

} // namespace rosbag_panel
