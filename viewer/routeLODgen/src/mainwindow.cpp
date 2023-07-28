#include    "mainwindow.h"
#include    "ui_mainwindow.h"

#include    <osgDB/ReadFile>
#include    <osgDB/WriteFile>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    std::string path = "../routes/experimental-polygon/models/tracks/1track.dmd";

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(path);

    osgDB::writeNodeFile(*model.get(), "1track.dmd");

    connect(ui->actionQuit, &QAction::triggered,
            this, &MainWindow::slotOnQuit);

    ui->twFilesList->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
MainWindow::~MainWindow()
{
    delete ui;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void MainWindow::slotOnQuit()
{
    QApplication::quit();
}

