#ifndef DATALISTMODEL_H
#define DATALISTMODEL_H
#include <QAbstractTableModel>
#include <QLinkedList>
#include <QVector>
class DataListModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    explicit DataListModel(QObject *parent = 0);
    ~DataListModel();
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QVariant headerData(int section, Qt::Orientation orientation,
                             int role = Qt::DisplayRole) const;
    Qt::ItemFlags flags(const QModelIndex & index) const;
    void sort(int column, Qt::SortOrder order);
    static int sortColumn;
    static Qt::SortOrder sortOrder;
    QList<quint32> getHashKeyForRows(const QList<int>&);
    void getKey(const QModelIndex &index,QString&,quint32& key);
private:
    QList<QVector<QVariant>> dataCache;

};

#endif // DATALISTMODEL_H
