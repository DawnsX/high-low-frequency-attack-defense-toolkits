#include "about.h"
#include "ui_about.h"

About::About(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::About)
{
    ui->setupUi(this);

    setWindowTitle("Unicorn EMV Reader v0.3 Serial Port Version");
    setWindowIcon(QIcon(":/images/favicon1.ico"));

    connect(ui->OK_Button, SIGNAL(clicked()), this, SLOT(on_OK()));
    connect(ui->Home_Button, SIGNAL(clicked()), this, SLOT(open_url()));

    ui->symbol->setPixmap(QPixmap(":/images/unicorn150.png"));
    ui->symbol->show();

    QString mess = "Version : 0.3 \nCompile Time: \n";
    mess.append(__DATE__);
    mess.append(" - ");
    mess.append(__TIME__);
    QFont msyhmono("Microsoft YaHei Mono", 10);
    ui->mess_label->setText(mess);
    ui->mess_label->setFont(msyhmono);
    ui->mess_label->show();

}

About::~About()
{
    delete ui;
}

void About::on_OK()
{
    hide();
}

void About::open_url()
{
    QDesktopServices::openUrl(QUrl("http://unicorn.360.cn/"));
}
