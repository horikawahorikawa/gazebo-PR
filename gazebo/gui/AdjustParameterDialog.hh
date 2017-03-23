//AdjustParameterDialog.h

#ifndef ADJUSTPARAMETERDIALOG_H_
#define ADJUSTPARAMETERDIALOG_H_

#include <QDialog>
#include <QValidator>

class QLabel;
class QPushButton;
class QLineEdit;
class QString;
class QValidator;
class QDoubleValidator;

class AdjustParameterDialog : public QDialog
{
	Q_OBJECT
public:
	AdjustParameterDialog(QWidget* parent = 0);
	void getLineEditText(QString* str0, QString* str1, QString* str2, QString* str3);
	void doubleEditTextChanged(const QString& text, QLineEdit* editor);
signals:
	void okButtonClicked();
private:
	QLabel* label0;
	QLabel* label1;
	QLabel* label2;
	QLabel* label3;
	QDoubleValidator* dvalidator;
	QPushButton* okButton;
	QPushButton* cancelButton;
	QLineEdit* editor0;
	QLineEdit* editor1;
	QLineEdit* editor2;
	QLineEdit* editor3;
};

#endif
