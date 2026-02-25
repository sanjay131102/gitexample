  QTimer* m_3dUpdateTimer = nullptr;
  Esri::ArcGISRuntime::GraphicsOverlay* m_aircraftOverlay3D = nullptr;

  QMap<int, Esri::ArcGISRuntime::Graphic*> m_vehicleGraphics_3d;
  QMap<int, Esri::ArcGISRuntime::ModelSceneSymbol*> m_vehicleSymbols_3d;

void MapPaintWidget::updateAircraft3D()
{
    using namespace Esri::ArcGISRuntime;

    if (MapProjection != 1)
        return;

    if (!m_aircraftOverlay3D)
        return;

    const QList<SimConnectAircraft>& aircraftList = getAiAircraft();

    const SimConnectAircraft* target = nullptr;

    // ---- Find target id 1 only ----
    for (const SimConnectAircraft& v : aircraftList)
    {
        if (v.getObjectId() == 1)
        {
            target = &v;
            break;
        }
    }

    if (!target)
        return;

    const Pos& pos = target->getPosition();
    if (!pos.isValid())
        return;

    double lon = pos.getLonX();
    double lat = pos.getLatY();
    double alt = pos.getAltitude();

    if (!std::isfinite(lon) || !std::isfinite(lat) || !std::isfinite(alt))
        return;

    int id = 1;

    // ---- Compute heading ----
    double heading = 0.0;
    if (target->getvx() != 0.0 || target->getvy() != 0.0)
    {
        heading = atan2(target->getvx(), target->getvy()) * 180.0 / M_PI;
        heading = fmod(heading + 360.0, 360.0);
    }

    Graphic* graphic = m_vehicleGraphics_3d.value(id, nullptr);

    // ================= CREATE ONCE =================
    if (!graphic)
    {
        QString modelPath = QDir::cleanPath(
            QDir(QCoreApplication::applicationDirPath())
                .filePath("../3d_data/drone2.dae"));

        if (!QFile::exists(modelPath))
            return;

        auto* symbol = new ModelSceneSymbol(
            QUrl::fromLocalFile(modelPath), 50.0);

        Point pt(lon, lat, alt, m_wgs84);

        graphic = new Graphic(pt, symbol);

        m_aircraftOverlay3D->graphics()->append(graphic);

        m_vehicleGraphics_3d.insert(id, graphic);
        m_vehicleSymbols_3d.insert(id, symbol);

        return;
    }

    // ================= UPDATE =================
    graphic->setGeometry(Point(lon, lat, alt, m_wgs84));

    ModelSceneSymbol* symbol = m_vehicleSymbols_3d.value(id, nullptr);
    if (symbol)
        symbol->setHeading(heading);
}

void MapPaintLayer::updateLayers()
{

        qDebug() << Q_FUNC_INFO;
        mapPainterMark->render();
        mapPainterTrack->render();
        mapPainterAircraft->render();
        mapPaintWidget->init3DSystem();


  // if(noRender())
  //   mapLayerEffective = mapLayer = mapLayerRoute = nullptr;
  // else
  // {
  //   float distKm = static_cast<float>(mapPaintWidget->distance());
  //   // Get the uncorrected effective layer - route painting is independent of declutter
  //   mapLayerEffective = layers->getLayer(distKm);
  //   mapLayer = layers->getLayer(distKm, detailLevel);
  //   mapLayerRoute = layers->getLayer(distKm, detailLevel + 1);
  // }
}

