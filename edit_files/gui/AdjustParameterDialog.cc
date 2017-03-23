#include <QtGui>
#include "AdjustParameterDialog.hh"

AdjustParameterDialog::AdjustParameterDialog(QWidget* parent) : QDialog(parent)
{
	label0 = new QLabel(tr("FOV") );
	label1 = new QLabel(tr("roll") );
	label2 = new QLabel(tr("pitch") );
	label3 = new QLabel(tr("yaw") );
	okButton = new QPushButton(tr("&OK") );
	cancelButton = new QPushButton(tr("&Cancel") );
	editor0 = new QLineEdit;
	editor1 = new QLineEdit;
	editor2 = new QLineEdit;
	editor3 = new QLineEdit;
	dvalidator = new QDoubleValidator;

	QHBoxLayout* layout = new QHBoxLayout;
	layout->addWidget(label0);
	layout->addWidget(editor0);
	layout->addWidget(label1);
	layout->addWidget(editor1);
	layout->addWidget(label2);
	layout->addWidget(editor2);
	layout->addWidget(label3);
	layout->addWidget(editor3);
	layout->addWidget(okButton);
	layout->addWidget(cancelButton);
	setLayout(layout);

	connect(okButton,SIGNAL(clicked()),this,SIGNAL(okButtonClicked()) );
	connect(okButton,SIGNAL(clicked()), this, SLOT(close()) );
	connect(cancelButton,SIGNAL(clicked()), this, SLOT(close()) );
	connect(editor0, SIGNAL(textChanged(const QString&)), this, SLOT(doubleEditTextChanged(const QString&, &editor0)));
	connect(editor1, SIGNAL(textChanged(const QString&)), this, SLOT(doubleEditTextChanged(const QString&, &editor1)));
	connect(editor2, SIGNAL(textChanged(const QString&)), this, SLOT(doubleEditTextChanged(const QString&, &editor2)));
	connect(editor3, SIGNAL(textChanged(const QString&)), this, SLOT(doubleEditTextChanged(const QString&, &editor3)));
	editor0->setValidator(dvalidator);
	editor1->setValidator(dvalidator);
	editor2->setValidator(dvalidator);
	editor3->setValidator(dvalidator);
}
void AdjustParameterDialog::doubleEditTextChanged(const QString &text, QLineEdit* editor) {
    QString str = text;
    int position = 0;
    QDoubleValidator* validator = (QDoubleValidator*)editor->validator();
    QValidator::State state = editor->validator()->validate(str, position);
}
void AdjustParameterDialog::getLineEditText(QString* qstr0, QString* qstr1, QString* qstr2, QString* qstr3)
{
	*qstr0 = editor0->text();
	*qstr1 = editor1->text();
	*qstr2 = editor2->text();
	*qstr3 = editor3->text();
}
