// In header — new member
Esri::ArcGISRuntime::SpatialReference m_wgs84;

else if (mapPaintWidget->MapProjection == 1) // 3D drone
{
    if (vehicle.isUser())
        return;

    const Pos& pos = vehicle.getPosition();
    if (!pos.isValid())
        return;

    double lon = pos.getLonX();
    double lat = pos.getLatY();
    double alt = pos.getAltitude();

    if (!std::isfinite(lon) || !std::isfinite(lat) || !std::isfinite(alt))
        return;

    int id = vehicle.getObjectId();

    auto* sceneView = mapPaintWidget->getMapSceneView();
    if (!sceneView || sceneView->graphicsOverlays()->size() <= 1)
        return;

    auto* overlay = sceneView->graphicsOverlays()->at(1);
    if (!overlay)
        return;

    // FIX 1: Cache wgs84 once per widget, not per call
    if (!mapPaintWidget->m_wgs84.isValid())
        mapPaintWidget->m_wgs84 = Esri::ArcGISRuntime::SpatialReference::wgs84();

    auto* aircraftGraphic =
        mapPaintWidget->m_vehicleGraphics_3d.value(id, nullptr);

    // ---------- CREATE MODEL (ONCE) ----------
    if (!aircraftGraphic)
    {
        QString modelPath = QDir::cleanPath(
            QDir(QCoreApplication::applicationDirPath())
                .filePath("../3d_data/drone2.dae"));

        if (!QFile::exists(modelPath))
            return;

        auto* modelSymbol =
            new Esri::ArcGISRuntime::ModelSceneSymbol(
                QUrl::fromLocalFile(modelPath), 50.0);

        modelSymbol->setAnchorPosition(
            Esri::ArcGISRuntime::SceneSymbolAnchorPosition::Bottom);

        aircraftGraphic = new Esri::ArcGISRuntime::Graphic(
            Esri::ArcGISRuntime::Point(
                lon, lat, alt,
                mapPaintWidget->m_wgs84),   // FIX 1: use cached ref
            modelSymbol);

        aircraftGraphic->attributes()->insertAttribute("ANGLE", heading);

        overlay->graphics()->append(aircraftGraphic);
        mapPaintWidget->m_vehicleGraphics_3d.insert(id, aircraftGraphic);

        if (!overlay->renderer())
        {
            auto* renderer =
                new Esri::ArcGISRuntime::SimpleRenderer(modelSymbol);

            Esri::ArcGISRuntime::RendererSceneProperties props;
            props.setHeadingExpression("[ANGLE]");
            renderer->setSceneProperties(props);
            overlay->setRenderer(renderer);
        }
    }

    // ---------- STATE UPDATE (UDP SIDE) ----------
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    auto& state = mapPaintWidget->m_vehicleState[id];

    if (!state.valid)
    {
        state.interpLat    = lat;
        state.interpLon    = lon;
        state.interpAlt    = alt;
        state.lastRenderTime = 0;
        state.valid        = true;
    }

    state.currLat = lat;
    state.currLon = lon;
    state.currAlt = alt;
    state.vx      = vehicle.getvx();
    state.vy      = vehicle.getvy();
    state.vz      = vehicle.getvz();
    state.cosLat  = std::cos(lat * M_PI / 180.0);  // FIX 2: update every packet, not just init
    state.lastUdpTime = now;

    // FIX 3: Update heading on main thread to avoid data race with timer lambda
    QMetaObject::invokeMethod(mapPaintWidget, [aircraftGraphic, heading]() {
        if (aircraftGraphic)
            aircraftGraphic->attributes()->replaceAttribute("ANGLE", heading);
    }, Qt::QueuedConnection);

    // ---------- TIMER (RENDER HEARTBEAT) ----------
    if (!mapPaintWidget->m_updateTimer)
    {
        mapPaintWidget->m_updateTimer = new QTimer(mapPaintWidget);
        mapPaintWidget->m_updateTimer->setInterval(16); // ~60 FPS

        QObject::connect(
            mapPaintWidget->m_updateTimer, &QTimer::timeout,
            mapPaintWidget, [this]() {

                qint64 tNow = QDateTime::currentMSecsSinceEpoch();
                const qint64 STALE_TIMEOUT_MS = 5000;

                auto* sceneView = this->mapPaintWidget->getMapSceneView();
                if (!sceneView || sceneView->graphicsOverlays()->size() <= 1)
                    return;

                auto* overlay = sceneView->graphicsOverlays()->at(1);

                auto it = this->mapPaintWidget->m_vehicleGraphics_3d.begin();
                while (it != this->mapPaintWidget->m_vehicleGraphics_3d.end())
                {
                    int    id      = it.key();
                    auto*  graphic = it.value();
                    auto&  state   = this->mapPaintWidget->m_vehicleState[id];

                    // ---------- CLEANUP ----------
                    if (!state.valid ||
                        (tNow - state.lastUdpTime) > STALE_TIMEOUT_MS)
                    {
                        overlay->graphics()->removeOne(graphic);
                        delete graphic;
                        it = this->mapPaintWidget->m_vehicleGraphics_3d.erase(it);
                        this->mapPaintWidget->m_vehicleState.remove(id);
                        continue;
                    }

                    // ---------- RENDER DT ----------
                    if (state.lastRenderTime == 0)
                    {
                        state.lastRenderTime = tNow;
                        ++it;
                        continue;
                    }

                    double dt =
                        std::clamp(
                            (tNow - state.lastRenderTime) / 1000.0,
                            0.0, 0.2);

                    state.lastRenderTime = tNow;

                    // ---------- SMOOTH + PREDICT ----------
                    const double blend = 0.15;

                    state.interpLat += (state.currLat - state.interpLat) * blend;
                    state.interpLon += (state.currLon - state.interpLon) * blend;
                    state.interpAlt += (state.currAlt - state.interpAlt) * blend;

                    double predLat = state.interpLat + (state.vy * dt) / 111000.0;
                    double predLon = state.interpLon + (state.vx * dt) /
                                                           (111000.0 * state.cosLat);
                    double predAlt = state.interpAlt + state.vz * dt;

                    // FIX 1: use cached wgs84, no per-frame construction
                    graphic->setGeometry(
                        Esri::ArcGISRuntime::Point(
                            predLon, predLat, predAlt,
                            this->mapPaintWidget->m_wgs84));

                    ++it;
                }
            });

        mapPaintWidget->m_updateTimer->start();
    }
}
