/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTimeEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QWidget *tab_manager;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_12;
    QGridLayout *gridLayout_3;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QTabWidget *tabWidget;
    QWidget *game_paly;
    QVBoxLayout *verticalLayout_4;
    QTimeEdit *game_time;
    QComboBox *game_choose;
    QCheckBox *teams_bool;
    QCheckBox *ai_bool;
    QPushButton *setup_game;
    QCheckBox *robotAcheck;
    QCheckBox *robotBcheck;
    QCheckBox *robotCcheck;
    QCheckBox *robotDcheck;
    QCheckBox *robotEcheck;
    QCheckBox *robotFcheck;
    QPushButton *start;
    QPushButton *pause;
    QPushButton *stop;
    QSpacerItem *verticalSpacer_2;
    QWidget *calibration;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout_2;
    QSpinBox *c2_x;
    QSpinBox *c2_y;
    QLabel *label;
    QHBoxLayout *horizontalLayout_3;
    QSpinBox *c3_x;
    QSpinBox *c3_y;
    QHBoxLayout *horizontalLayout;
    QSpinBox *c1_x;
    QSpinBox *c1_y;
    QHBoxLayout *horizontalLayout_4;
    QSpinBox *c4_x;
    QSpinBox *c4_y;
    QLabel *calibInfo;
    QSpinBox *threshold;
    QSpacerItem *verticalSpacer;
    QPushButton *calibrate;
    QPushButton *quit_button;
    QLabel *calib_result;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(944, 706);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        tab_manager = new QWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        verticalLayout_2 = new QVBoxLayout(tab_manager);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_12 = new QGroupBox(tab_manager);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_12->sizePolicy().hasHeightForWidth());
        groupBox_12->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_12);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        frame = new QFrame(groupBox_12);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy1);
        frame->setMaximumSize(QSize(300, 16777215));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        tabWidget = new QTabWidget(frame);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        game_paly = new QWidget();
        game_paly->setObjectName(QString::fromUtf8("game_paly"));
        verticalLayout_4 = new QVBoxLayout(game_paly);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        game_time = new QTimeEdit(game_paly);
        game_time->setObjectName(QString::fromUtf8("game_time"));
        game_time->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(game_time);

        game_choose = new QComboBox(game_paly);
        game_choose->setObjectName(QString::fromUtf8("game_choose"));

        verticalLayout_4->addWidget(game_choose);

        teams_bool = new QCheckBox(game_paly);
        teams_bool->setObjectName(QString::fromUtf8("teams_bool"));

        verticalLayout_4->addWidget(teams_bool);

        ai_bool = new QCheckBox(game_paly);
        ai_bool->setObjectName(QString::fromUtf8("ai_bool"));

        verticalLayout_4->addWidget(ai_bool);

        setup_game = new QPushButton(game_paly);
        setup_game->setObjectName(QString::fromUtf8("setup_game"));

        verticalLayout_4->addWidget(setup_game);

        robotAcheck = new QCheckBox(game_paly);
        robotAcheck->setObjectName(QString::fromUtf8("robotAcheck"));

        verticalLayout_4->addWidget(robotAcheck);

        robotBcheck = new QCheckBox(game_paly);
        robotBcheck->setObjectName(QString::fromUtf8("robotBcheck"));

        verticalLayout_4->addWidget(robotBcheck);

        robotCcheck = new QCheckBox(game_paly);
        robotCcheck->setObjectName(QString::fromUtf8("robotCcheck"));

        verticalLayout_4->addWidget(robotCcheck);

        robotDcheck = new QCheckBox(game_paly);
        robotDcheck->setObjectName(QString::fromUtf8("robotDcheck"));

        verticalLayout_4->addWidget(robotDcheck);

        robotEcheck = new QCheckBox(game_paly);
        robotEcheck->setObjectName(QString::fromUtf8("robotEcheck"));

        verticalLayout_4->addWidget(robotEcheck);

        robotFcheck = new QCheckBox(game_paly);
        robotFcheck->setObjectName(QString::fromUtf8("robotFcheck"));

        verticalLayout_4->addWidget(robotFcheck);

        start = new QPushButton(game_paly);
        start->setObjectName(QString::fromUtf8("start"));

        verticalLayout_4->addWidget(start);

        pause = new QPushButton(game_paly);
        pause->setObjectName(QString::fromUtf8("pause"));

        verticalLayout_4->addWidget(pause);

        stop = new QPushButton(game_paly);
        stop->setObjectName(QString::fromUtf8("stop"));

        verticalLayout_4->addWidget(stop);

        verticalSpacer_2 = new QSpacerItem(20, 150, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_2);

        tabWidget->addTab(game_paly, QString());
        calibration = new QWidget();
        calibration->setObjectName(QString::fromUtf8("calibration"));
        verticalLayout_7 = new QVBoxLayout(calibration);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        groupBox = new QGroupBox(calibration);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        c2_x = new QSpinBox(groupBox);
        c2_x->setObjectName(QString::fromUtf8("c2_x"));
        c2_x->setAlignment(Qt::AlignCenter);
        c2_x->setMaximum(1279);

        horizontalLayout_2->addWidget(c2_x);

        c2_y = new QSpinBox(groupBox);
        c2_y->setObjectName(QString::fromUtf8("c2_y"));
        c2_y->setAlignment(Qt::AlignCenter);
        c2_y->setMaximum(719);

        horizontalLayout_2->addWidget(c2_y);


        gridLayout->addLayout(horizontalLayout_2, 5, 0, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 3, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        c3_x = new QSpinBox(groupBox);
        c3_x->setObjectName(QString::fromUtf8("c3_x"));
        c3_x->setAlignment(Qt::AlignCenter);
        c3_x->setMaximum(1279);

        horizontalLayout_3->addWidget(c3_x);

        c3_y = new QSpinBox(groupBox);
        c3_y->setObjectName(QString::fromUtf8("c3_y"));
        c3_y->setAlignment(Qt::AlignCenter);
        c3_y->setMaximum(719);

        horizontalLayout_3->addWidget(c3_y);


        gridLayout->addLayout(horizontalLayout_3, 6, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        c1_x = new QSpinBox(groupBox);
        c1_x->setObjectName(QString::fromUtf8("c1_x"));
        c1_x->setAlignment(Qt::AlignCenter);
        c1_x->setMaximum(1279);

        horizontalLayout->addWidget(c1_x);

        c1_y = new QSpinBox(groupBox);
        c1_y->setObjectName(QString::fromUtf8("c1_y"));
        c1_y->setAlignment(Qt::AlignCenter);
        c1_y->setMaximum(719);

        horizontalLayout->addWidget(c1_y);


        gridLayout->addLayout(horizontalLayout, 4, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        c4_x = new QSpinBox(groupBox);
        c4_x->setObjectName(QString::fromUtf8("c4_x"));
        c4_x->setAlignment(Qt::AlignCenter);
        c4_x->setMaximum(1279);

        horizontalLayout_4->addWidget(c4_x);

        c4_y = new QSpinBox(groupBox);
        c4_y->setObjectName(QString::fromUtf8("c4_y"));
        c4_y->setAlignment(Qt::AlignCenter);
        c4_y->setMaximum(719);

        horizontalLayout_4->addWidget(c4_y);


        gridLayout->addLayout(horizontalLayout_4, 7, 0, 1, 1);

        calibInfo = new QLabel(groupBox);
        calibInfo->setObjectName(QString::fromUtf8("calibInfo"));

        gridLayout->addWidget(calibInfo, 0, 0, 1, 1);

        threshold = new QSpinBox(groupBox);
        threshold->setObjectName(QString::fromUtf8("threshold"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(threshold->sizePolicy().hasHeightForWidth());
        threshold->setSizePolicy(sizePolicy2);
        threshold->setAlignment(Qt::AlignCenter);
        threshold->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
        threshold->setMaximum(255);

        gridLayout->addWidget(threshold, 1, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 150, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 9, 0, 1, 1);

        calibrate = new QPushButton(groupBox);
        calibrate->setObjectName(QString::fromUtf8("calibrate"));
        calibrate->setEnabled(true);
        QSizePolicy sizePolicy3(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(calibrate->sizePolicy().hasHeightForWidth());
        calibrate->setSizePolicy(sizePolicy3);

        gridLayout->addWidget(calibrate, 8, 0, 1, 1);


        verticalLayout_7->addWidget(groupBox);

        tabWidget->addTab(calibration, QString());

        verticalLayout_3->addWidget(tabWidget);

        quit_button = new QPushButton(frame);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        sizePolicy3.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy3);

        verticalLayout_3->addWidget(quit_button);


        gridLayout_3->addWidget(frame, 1, 1, 1, 1);

        calib_result = new QLabel(groupBox_12);
        calib_result->setObjectName(QString::fromUtf8("calib_result"));

        gridLayout_3->addWidget(calib_result, 1, 0, 2, 1);


        verticalLayout_2->addWidget(groupBox_12);


        gridLayout_2->addWidget(tab_manager, 0, 0, 1, 1);

        MainWindowDesign->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        retranslateUi(MainWindowDesign);
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        groupBox_12->setTitle(QApplication::translate("MainWindowDesign", "Calibration result", 0, QApplication::UnicodeUTF8));
        game_time->setDisplayFormat(QApplication::translate("MainWindowDesign", "mm:ss", 0, QApplication::UnicodeUTF8));
        teams_bool->setText(QApplication::translate("MainWindowDesign", "Game with teams", 0, QApplication::UnicodeUTF8));
        ai_bool->setText(QApplication::translate("MainWindowDesign", "Play VS AI", 0, QApplication::UnicodeUTF8));
        setup_game->setText(QApplication::translate("MainWindowDesign", "Setup game + Auto detect", 0, QApplication::UnicodeUTF8));
        robotAcheck->setText(QApplication::translate("MainWindowDesign", "Robot A", 0, QApplication::UnicodeUTF8));
        robotBcheck->setText(QApplication::translate("MainWindowDesign", "Robot B", 0, QApplication::UnicodeUTF8));
        robotCcheck->setText(QApplication::translate("MainWindowDesign", "Robot C", 0, QApplication::UnicodeUTF8));
        robotDcheck->setText(QApplication::translate("MainWindowDesign", "Robot D", 0, QApplication::UnicodeUTF8));
        robotEcheck->setText(QApplication::translate("MainWindowDesign", "Robot E", 0, QApplication::UnicodeUTF8));
        robotFcheck->setText(QApplication::translate("MainWindowDesign", "Robot F", 0, QApplication::UnicodeUTF8));
        start->setText(QApplication::translate("MainWindowDesign", "Start", 0, QApplication::UnicodeUTF8));
        pause->setText(QApplication::translate("MainWindowDesign", "Pause", 0, QApplication::UnicodeUTF8));
        stop->setText(QApplication::translate("MainWindowDesign", "Stop", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(game_paly), QApplication::translate("MainWindowDesign", "Game play", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QString());
        label->setText(QApplication::translate("MainWindowDesign", "Corners (x,y) - Manual calibration", 0, QApplication::UnicodeUTF8));
        calibInfo->setText(QApplication::translate("MainWindowDesign", "                 Choose the threshold - Auto calibration", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        calibrate->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        calibrate->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        calibrate->setText(QApplication::translate("MainWindowDesign", "Calibrate", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(calibration), QApplication::translate("MainWindowDesign", "Calibration", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
        calib_result->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
