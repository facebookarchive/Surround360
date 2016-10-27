#include "PreviewCamComboBox.hpp"
#include "PointGrey.hpp"
#include <string>
#include <iostream>
#include <memory>
#include "CameraController.hpp"
#include "Config.hpp"

using namespace surround360;
using namespace Gtk;
using namespace std;

PreviewCamComboBox::PreviewCamComboBox()
  : Gtk::ComboBox() {
  m_refTreeModel = ListStore::create(m_model);
  set_model(m_refTreeModel);
}

void PreviewCamComboBox::reload() {
  SerialIndexVector serials;

  m_refTreeModel->clear();

  const unsigned int n = PointGreyCamera::findCameras();
  for (unsigned int k = 0; k < n; ++k) {
    PointGreyCameraPtr c = PointGreyCamera::getCamera(k);
    serials.push_back(std::make_tuple(k, c->getSerialNumber(), 0));
  }

  sort(serials.begin(), serials.end(),
       [](const SerialIndexTuple& left, const SerialIndexTuple& right)
       { return get<1>(left) < get<1>(right); });

  for (auto m = 0; m < serials.size(); ++m) {
    TreeModel::Row row = *(m_refTreeModel->append());
    row[m_model.m_num] = m;
    row[m_model.m_busNum] = get<1>(serials[m]);

    if (m == 0) {
      set_active(row);
    }
  }

  pack_start(m_model.m_num);
}

void PreviewCamComboBox::on_changed() {
  cout << "Active preview cam changed" << endl;
  try {
    CameraController& ctl = CameraController::get();
    auto iter = get_active_row_number();
    auto& cfg = CameraConfig::get();

    cfg.previewCam = iter;
    ctl.setPreviewCamera(iter);
  } catch (...) {
    cout << "Error when setting a preview camera" << endl;
  }
}
