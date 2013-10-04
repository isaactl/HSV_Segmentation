#include <QtGui>
#include "SetRegionRGB.h"

SetRegionRGB::SetRegionRGB( QWidget *parent )
    :QDialog(parent)
{
    QSDistanceThreshold = 0.025;
    QSPointColorThreshold = 6;
    QSRegionColorThreshold = 6;
    QSMinClusterSize = 400;

    DistanceThreshold = new QLabel(tr("DistanceThreshold"));
    DistanceThreshold_lineEdit = new QLineEdit;
    DistanceThreshold->setBuddy(DistanceThreshold_lineEdit);
    DistanceThreshold_lineEdit->setText( tr("0.025") );

    PointColorThreshold = new QLabel(tr("PointColorThreshold"));
    PointColorThreshold_lineEdit = new QLineEdit;
    PointColorThreshold->setBuddy(PointColorThreshold_lineEdit);
    PointColorThreshold_lineEdit->setText( tr("6") );

    RegionColorThreshold = new QLabel(tr("RegionColorThreshold"));
    RegionColorThreshold_lineEdit = new QLineEdit;
    RegionColorThreshold->setBuddy(RegionColorThreshold_lineEdit);
    RegionColorThreshold_lineEdit->setText( tr("5") );

    MinClusterSize = new QLabel(tr("MinClusterSize"));
    MinClusterSize_lineEdit = new QLineEdit;
    MinClusterSize->setBuddy(MinClusterSize_lineEdit);
    MinClusterSize_lineEdit->setText( tr("400") );


    okButton = new QPushButton(tr("&Set"));
    connect(okButton, SIGNAL(clicked()),
            this, SLOT(SetValue()));

    cancelButton = new QPushButton(tr("&Cancle"));
    connect(cancelButton, SIGNAL(clicked()),
            this, SLOT(close()));

    // ²¼¾Ö
    QHBoxLayout *first = new QHBoxLayout;
    first->addWidget(DistanceThreshold);
    first->addWidget(DistanceThreshold_lineEdit);

    QHBoxLayout *second = new QHBoxLayout;
    second->addWidget(PointColorThreshold);
    second->addWidget(PointColorThreshold_lineEdit);

    QHBoxLayout *third = new QHBoxLayout;
    third->addWidget(RegionColorThreshold);
    third->addWidget(RegionColorThreshold_lineEdit);

    QHBoxLayout *fourth = new QHBoxLayout;
    fourth->addWidget(MinClusterSize);
    fourth->addWidget(MinClusterSize_lineEdit);

    QHBoxLayout *fifth = new QHBoxLayout;
    fifth->addWidget(okButton);
    fifth->addWidget(cancelButton);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(first);
    mainLayout->addLayout(second);
    mainLayout->addLayout(third);
    mainLayout->addLayout(fourth);
    mainLayout->addLayout(fifth);

    setLayout(mainLayout);
    setWindowTitle(tr("Set RGB Parameter"));
    setFixedHeight(sizeHint().height());

}

void SetRegionRGB::SetValue()
{
    QSDistanceThreshold = DistanceThreshold_lineEdit->text().toFloat();
    QSPointColorThreshold = PointColorThreshold_lineEdit->text().toUInt();
    QSRegionColorThreshold = RegionColorThreshold_lineEdit->text().toUInt();
    QSMinClusterSize = MinClusterSize_lineEdit->text().toUInt();
    emit RegionGrowing(QSDistanceThreshold,QSPointColorThreshold,QSRegionColorThreshold,QSMinClusterSize);
}
