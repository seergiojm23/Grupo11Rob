/****************************************************************************
** Meta object code from reading C++ file 'specificworker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../specificworker.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'specificworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SpecificWorker_t {
    const uint offsetsAndSize[44];
    char stringdata0[272];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_SpecificWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_SpecificWorker_t qt_meta_stringdata_SpecificWorker = {
    {
QT_MOC_LITERAL(0, 14), // "SpecificWorker"
QT_MOC_LITERAL(15, 10), // "initialize"
QT_MOC_LITERAL(26, 0), // ""
QT_MOC_LITERAL(27, 21), // "new_mouse_coordinates"
QT_MOC_LITERAL(49, 11), // "update_grid"
QT_MOC_LITERAL(61, 28), // "std::vector<Eigen::Vector2f>"
QT_MOC_LITERAL(90, 12), // "ldata_bpearl"
QT_MOC_LITERAL(103, 10), // "clear_grid"
QT_MOC_LITERAL(114, 17), // "dijkstraAlgorithm"
QT_MOC_LITERAL(132, 20), // "std::vector<QPointF>"
QT_MOC_LITERAL(153, 12), // "GridPosition"
QT_MOC_LITERAL(166, 5), // "start"
QT_MOC_LITERAL(172, 6), // "target"
QT_MOC_LITERAL(179, 9), // "draw_path"
QT_MOC_LITERAL(189, 15), // "vector<QPointF>"
QT_MOC_LITERAL(205, 4), // "path"
QT_MOC_LITERAL(210, 15), // "QGraphicsScene*"
QT_MOC_LITERAL(226, 5), // "scene"
QT_MOC_LITERAL(232, 7), // "compute"
QT_MOC_LITERAL(240, 9), // "emergency"
QT_MOC_LITERAL(250, 7), // "restore"
QT_MOC_LITERAL(258, 13) // "startup_check"

    },
    "SpecificWorker\0initialize\0\0"
    "new_mouse_coordinates\0update_grid\0"
    "std::vector<Eigen::Vector2f>\0ldata_bpearl\0"
    "clear_grid\0dijkstraAlgorithm\0"
    "std::vector<QPointF>\0GridPosition\0"
    "start\0target\0draw_path\0vector<QPointF>\0"
    "path\0QGraphicsScene*\0scene\0compute\0"
    "emergency\0restore\0startup_check"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SpecificWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   74,    2, 0x0a,    1 /* Public */,
       3,    1,   75,    2, 0x0a,    2 /* Public */,
       4,    1,   78,    2, 0x0a,    4 /* Public */,
       7,    0,   81,    2, 0x0a,    6 /* Public */,
       8,    2,   82,    2, 0x0a,    7 /* Public */,
      13,    2,   87,    2, 0x0a,   10 /* Public */,
      18,    0,   92,    2, 0x0a,   13 /* Public */,
      19,    0,   93,    2, 0x0a,   14 /* Public */,
      20,    0,   94,    2, 0x0a,   15 /* Public */,
      21,    0,   95,    2, 0x0a,   16 /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QPointF,    2,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    0x80000000 | 9, 0x80000000 | 10, 0x80000000 | 10,   11,   12,
    QMetaType::Void, 0x80000000 | 14, 0x80000000 | 16,   15,   17,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Int,

       0        // eod
};

void SpecificWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SpecificWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->initialize(); break;
        case 1: _t->new_mouse_coordinates((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 2: _t->update_grid((*reinterpret_cast< std::add_pointer_t<std::vector<Eigen::Vector2f>>>(_a[1]))); break;
        case 3: _t->clear_grid(); break;
        case 4: { std::vector<QPointF> _r = _t->dijkstraAlgorithm((*reinterpret_cast< std::add_pointer_t<GridPosition>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<GridPosition>>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::vector<QPointF>*>(_a[0]) = std::move(_r); }  break;
        case 5: _t->draw_path((*reinterpret_cast< std::add_pointer_t<vector<QPointF>>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QGraphicsScene*>>(_a[2]))); break;
        case 6: _t->compute(); break;
        case 7: _t->emergency(); break;
        case 8: _t->restore(); break;
        case 9: { int _r = _t->startup_check();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QGraphicsScene* >(); break;
            }
            break;
        }
    }
}

const QMetaObject SpecificWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<GenericWorker::staticMetaObject>(),
    qt_meta_stringdata_SpecificWorker.offsetsAndSize,
    qt_meta_data_SpecificWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_SpecificWorker_t
, QtPrivate::TypeAndForceComplete<SpecificWorker, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<QPointF, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const std::vector<Eigen::Vector2f> &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<std::vector<QPointF>, std::false_type>, QtPrivate::TypeAndForceComplete<GridPosition, std::false_type>, QtPrivate::TypeAndForceComplete<GridPosition, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const vector<QPointF> &, std::false_type>, QtPrivate::TypeAndForceComplete<QGraphicsScene *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>


>,
    nullptr
} };


const QMetaObject *SpecificWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SpecificWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SpecificWorker.stringdata0))
        return static_cast<void*>(this);
    return GenericWorker::qt_metacast(_clname);
}

int SpecificWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = GenericWorker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
