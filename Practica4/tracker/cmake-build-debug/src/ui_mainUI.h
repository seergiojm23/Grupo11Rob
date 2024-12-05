/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QVBoxLayout *verticalLayout_2;
    QSplitter *splitter_2;
    QSplitter *splitter;
    QFrame *frame;
    QFrame *frame_dist;
    QFrame *frame_2;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QLCDNumber *lcdNumber_dist_to_person;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLCDNumber *lcdNumber_angle_to_person;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLCDNumber *lcdNumber_adv;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLCDNumber *lcdNumber_rot;
    QHBoxLayout *horizontalLayout;
    QLabel *label_state_name;
    QLabel *label_state;
    QHBoxLayout *horizontalLayout_8;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pushButton_adapt;
    QPushButton *pushButton_stop;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName(QString::fromUtf8("guiDlg"));
        guiDlg->resize(1044, 678);
        verticalLayout_2 = new QVBoxLayout(guiDlg);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        splitter_2 = new QSplitter(guiDlg);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Horizontal);
        splitter = new QSplitter(splitter_2);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Vertical);
        frame = new QFrame(splitter);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(600, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        splitter->addWidget(frame);
        frame_dist = new QFrame(splitter);
        frame_dist->setObjectName(QString::fromUtf8("frame_dist"));
        frame_dist->setMaximumSize(QSize(16777215, 150));
        frame_dist->setFrameShape(QFrame::StyledPanel);
        frame_dist->setFrameShadow(QFrame::Raised);
        splitter->addWidget(frame_dist);
        splitter_2->addWidget(splitter);
        frame_2 = new QFrame(splitter_2);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frame_2->sizePolicy().hasHeightForWidth());
        frame_2->setSizePolicy(sizePolicy1);
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setSizeConstraint(QLayout::SetMinimumSize);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(frame_2);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);

        lcdNumber_dist_to_person = new QLCDNumber(frame_2);
        lcdNumber_dist_to_person->setObjectName(QString::fromUtf8("lcdNumber_dist_to_person"));
        QFont font;
        font.setBold(true);
        lcdNumber_dist_to_person->setFont(font);

        horizontalLayout_2->addWidget(lcdNumber_dist_to_person);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_2 = new QLabel(frame_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);

        lcdNumber_angle_to_person = new QLCDNumber(frame_2);
        lcdNumber_angle_to_person->setObjectName(QString::fromUtf8("lcdNumber_angle_to_person"));
        lcdNumber_angle_to_person->setFont(font);

        horizontalLayout_3->addWidget(lcdNumber_angle_to_person);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(frame_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_4->addWidget(label_4);

        lcdNumber_adv = new QLCDNumber(frame_2);
        lcdNumber_adv->setObjectName(QString::fromUtf8("lcdNumber_adv"));

        horizontalLayout_4->addWidget(lcdNumber_adv);


        verticalLayout_4->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(frame_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        lcdNumber_rot = new QLCDNumber(frame_2);
        lcdNumber_rot->setObjectName(QString::fromUtf8("lcdNumber_rot"));

        horizontalLayout_5->addWidget(lcdNumber_rot);


        verticalLayout_4->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_state_name = new QLabel(frame_2);
        label_state_name->setObjectName(QString::fromUtf8("label_state_name"));

        horizontalLayout->addWidget(label_state_name);

        label_state = new QLabel(frame_2);
        label_state->setObjectName(QString::fromUtf8("label_state"));

        horizontalLayout->addWidget(label_state);


        verticalLayout_4->addLayout(horizontalLayout);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));

        verticalLayout_4->addLayout(horizontalLayout_8);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));

        verticalLayout->addLayout(horizontalLayout_7);


        verticalLayout_4->addLayout(verticalLayout);

        pushButton_adapt = new QPushButton(frame_2);
        pushButton_adapt->setObjectName(QString::fromUtf8("pushButton_adapt"));
        pushButton_adapt->setCheckable(true);
        pushButton_adapt->setChecked(true);

        verticalLayout_4->addWidget(pushButton_adapt);

        pushButton_stop = new QPushButton(frame_2);
        pushButton_stop->setObjectName(QString::fromUtf8("pushButton_stop"));
        pushButton_stop->setCheckable(true);
        pushButton_stop->setChecked(true);

        verticalLayout_4->addWidget(pushButton_stop);


        verticalLayout_3->addLayout(verticalLayout_4);

        splitter_2->addWidget(frame_2);

        verticalLayout_2->addWidget(splitter_2);


        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QCoreApplication::translate("guiDlg", "Person Tracker", nullptr));
        label->setText(QCoreApplication::translate("guiDlg", "distance to person", nullptr));
        label_2->setText(QCoreApplication::translate("guiDlg", "angle to person", nullptr));
        label_4->setText(QCoreApplication::translate("guiDlg", "adv speed", nullptr));
        label_5->setText(QCoreApplication::translate("guiDlg", "rot speed", nullptr));
        label_state_name->setText(QCoreApplication::translate("guiDlg", "state:", nullptr));
        label_state->setText(QCoreApplication::translate("guiDlg", "TextLabel", nullptr));
        pushButton_adapt->setText(QCoreApplication::translate("guiDlg", "Adapt", nullptr));
        pushButton_stop->setText(QCoreApplication::translate("guiDlg", "Track", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
