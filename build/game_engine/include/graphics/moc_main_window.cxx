/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/game_engine/include/graphics/main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      36,   11,   11,   11, 0x05,
      81,   11,   11,   11, 0x05,
      93,   11,   11,   11, 0x05,
     104,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     120,   11,   11,   11, 0x0a,
     143,   11,   11,   11, 0x0a,
     162,   11,   11,   11, 0x0a,
     193,   11,   11,   11, 0x0a,
     212,   11,   11,   11, 0x0a,
     230,   11,   11,   11, 0x0a,
     257,  254,   11,   11, 0x0a,
     305,   11,   11,   11, 0x0a,
     319,   11,   11,   11, 0x0a,
     340,   11,   11,   11, 0x0a,
     385,  380,   11,   11, 0x0a,
     416,  411,   11,   11, 0x0a,
     447,  442,   11,   11, 0x0a,
     478,  473,   11,   11, 0x0a,
     509,  504,   11,   11, 0x0a,
     540,  535,   11,   11, 0x0a,
     571,  566,   11,   11, 0x0a,
     602,  597,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0tryCalibration(int64_t)\0"
    "manually_calibrate(std::vector<cv::Point2f>)\0"
    "calibrate()\0startROS()\0setFullscreen()\0"
    "on_calibrate_clicked()\0on_start_clicked()\0"
    "on_threshold_valueChanged(int)\0"
    "on_pause_clicked()\0on_stop_clicked()\0"
    "on_setup_game_clicked()\0,,\0"
    "canStart(bool,cv::Mat,std::vector<cv::Point2f>)\0"
    "update_time()\0reset_buttons_slot()\0"
    "detected_robots_slot(std::map<int,int>)\0"
    "c1_x\0on_c1_x_valueChanged(int)\0c1_y\0"
    "on_c1_y_valueChanged(int)\0c2_x\0"
    "on_c2_x_valueChanged(int)\0c2_y\0"
    "on_c2_y_valueChanged(int)\0c3_x\0"
    "on_c3_x_valueChanged(int)\0c3_y\0"
    "on_c3_y_valueChanged(int)\0c4_x\0"
    "on_c4_x_valueChanged(int)\0c4_y\0"
    "on_c4_y_valueChanged(int)\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->tryCalibration((*reinterpret_cast< int64_t(*)>(_a[1]))); break;
        case 1: _t->manually_calibrate((*reinterpret_cast< std::vector<cv::Point2f>(*)>(_a[1]))); break;
        case 2: _t->calibrate(); break;
        case 3: _t->startROS(); break;
        case 4: _t->setFullscreen(); break;
        case 5: _t->on_calibrate_clicked(); break;
        case 6: _t->on_start_clicked(); break;
        case 7: _t->on_threshold_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_pause_clicked(); break;
        case 9: _t->on_stop_clicked(); break;
        case 10: _t->on_setup_game_clicked(); break;
        case 11: _t->canStart((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< cv::Mat(*)>(_a[2])),(*reinterpret_cast< std::vector<cv::Point2f>(*)>(_a[3]))); break;
        case 12: _t->update_time(); break;
        case 13: _t->reset_buttons_slot(); break;
        case 14: _t->detected_robots_slot((*reinterpret_cast< std::map<int,int>(*)>(_a[1]))); break;
        case 15: _t->on_c1_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->on_c1_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_c2_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 18: _t->on_c2_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 19: _t->on_c3_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 20: _t->on_c3_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 21: _t->on_c4_x_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 22: _t->on_c4_y_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::tryCalibration(int64_t _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::manually_calibrate(std::vector<cv::Point2f> _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MainWindow::calibrate()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void MainWindow::startROS()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}

// SIGNAL 4
void MainWindow::setFullscreen()
{
    QMetaObject::activate(this, &staticMetaObject, 4, 0);
}
QT_END_MOC_NAMESPACE
