/****************************************************************************
** Meta object code from reading C++ file 'MapBuilderWifi.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/rtabmap/examples/WifiMapping/MapBuilderWifi.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MapBuilderWifi.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MapBuilderWifi_t {
    QByteArrayData data[5];
    char stringdata0[60];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MapBuilderWifi_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MapBuilderWifi_t qt_meta_stringdata_MapBuilderWifi = {
    {
QT_MOC_LITERAL(0, 0, 14), // "MapBuilderWifi"
QT_MOC_LITERAL(1, 15, 17), // "processStatistics"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 19), // "rtabmap::Statistics"
QT_MOC_LITERAL(4, 54, 5) // "stats"

    },
    "MapBuilderWifi\0processStatistics\0\0"
    "rtabmap::Statistics\0stats"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MapBuilderWifi[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void MapBuilderWifi::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MapBuilderWifi *_t = static_cast<MapBuilderWifi *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->processStatistics((*reinterpret_cast< const rtabmap::Statistics(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject MapBuilderWifi::staticMetaObject = {
    { &MapBuilder::staticMetaObject, qt_meta_stringdata_MapBuilderWifi.data,
      qt_meta_data_MapBuilderWifi,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MapBuilderWifi::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MapBuilderWifi::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MapBuilderWifi.stringdata0))
        return static_cast<void*>(const_cast< MapBuilderWifi*>(this));
    return MapBuilder::qt_metacast(_clname);
}

int MapBuilderWifi::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = MapBuilder::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
