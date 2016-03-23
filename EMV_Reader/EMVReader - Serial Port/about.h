#ifndef ABOUT_H
#define ABOUT_H

#include <QWidget>
#include <QFontDatabase>
#include <QDesktopServices>
#include <QUrl>

namespace Ui {
class About;
}

class About : public QWidget
{
    Q_OBJECT

public:
    explicit About(QWidget *parent = 0);
    ~About();

private slots:
    void on_OK();
    void open_url();

private:
    Ui::About *ui;
};

#endif // ABOUT_H
