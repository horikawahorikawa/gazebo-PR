/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelEditor.hh"
#include "gazebo/gui/model/ModelEditor_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelEditor_TEST::AddItemToPalette()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // verify we have a model editor widget
  gui::ModelEditor *modelEditor =
      dynamic_cast<gui::ModelEditor *>(mainWindow->GetEditor("model"));
  QVERIFY(modelEditor);

  // add a custom push button to the model editor palette
  QPushButton *testButton = new QPushButton("TEST_BUTTON");
  testButton->setObjectName("my_custom_test_button");
  modelEditor->AddItemToPalette(testButton, "test_categorty");

  QPushButton *retButton =
      mainWindow->findChild<QPushButton *>("my_custom_test_button");

  // verify that the push button is added.
  QVERIFY(retButton);
  QVERIFY(retButton->text().toStdString() ==  "TEST_BUTTON");

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditor_TEST)
