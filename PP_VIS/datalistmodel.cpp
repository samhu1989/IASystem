#include "datalistmodel.h"
#include "pipe.h"
#include <QString>
int DataListModel::sortColumn = 0;
Qt::SortOrder DataListModel::sortOrder = Qt::DescendingOrder;

DataListModel::DataListModel(QObject *parent)
{
    quint32 r;
    for(r=0;r<Pipe::_PipeData.size();++r)
    {
       dataCache.push_back(Pipe::parseDataHeadAt(r));
    }
}

DataListModel::~DataListModel()
{

}

int DataListModel::rowCount(const QModelIndex &parent) const
{
    return Pipe::_PipeData.size();
}

int DataListModel::columnCount(const QModelIndex &parent) const
{
    return 8;
}

QVariant DataListModel::data(const QModelIndex &index, int role) const
{
    if(!index.isValid())
    {
        return QVariant();
    }

    if(role==Qt::DisplayRole || role == Qt::ToolTipRole)
    {
        quint32 row = index.row();
        quint32 col = index.column();

        if(col<dataCache[row].size())
        {
            return dataCache[row][col];
        }else{
            return QVariant();
        }
    }
    return QVariant();
}

QVariant DataListModel::headerData(int section, Qt::Orientation orientation,
                         int role) const
{
    if( role != Qt::DisplayRole)return QVariant();
    if(orientation==Qt::Horizontal)
    {
        switch(section)
        {
        case 0:
            return QString("Hash Address");
        case 1:
            return QString("Type");
        case 2:
            return QString("Custom Info");
        default:
            return QVariant();
        }
    }
    if(orientation==Qt::Vertical)
    {
        return section;
    }
}

Qt::ItemFlags DataListModel::flags(const QModelIndex & index) const
{
    if(!index.isValid())
    {
        return Qt::ItemIsEnabled;
    }
    return QAbstractTableModel::flags(index);
}

bool lessThan(const QVector<QVariant>& a,const QVector<QVariant>& b)
{
    int c = DataListModel::sortColumn;
    if(b.size()<=c)
    {
        return true;
    }else if(a.size()<=c)
    {
        return false;
    }
    if(DataListModel::sortOrder == Qt::DescendingOrder)
    {
        return a[c].toString() < b[c].toString();
    }else{
        return a[c].toString() > b[c].toString();
    }
}

void DataListModel::sort(int column, Qt::SortOrder order)
{
    sortColumn = column;
    sortOrder = order;
    qSort(dataCache.begin(),dataCache.end(),lessThan);
}

QList<quint32> DataListModel::getHashKeyForRows(const QList<int>& list)
{
    QList<quint32> result;
    foreach(int r,list)
    {
        result.push_back(dataCache[r][0].toUInt());
    }
    return result;
}

void DataListModel::getKey(const QModelIndex &index,QString& info,quint32& key)
{
    int r = index.row();
    info = dataCache[r][1].toString();
    key = dataCache[r][0].toUInt();
}
