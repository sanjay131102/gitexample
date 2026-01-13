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

        Esri::ArcGISRuntime::GraphicsOverlay* overlay = nullptr;
        if (mapPaintWidget->getMapSceneView() &&
            mapPaintWidget->getMapSceneView()->graphicsOverlays()->size() > 1)
            overlay = mapPaintWidget->getMapSceneView()->graphicsOverlays()->at(1);
        if (!overlay)
            return;

        Esri::ArcGISRuntime::Graphic* aircraftGraphic =
            mapPaintWidget->m_vehicleGraphics_3d.value(id, nullptr);

        // --- Create model if missing ---
        if (!aircraftGraphic)
        {
            QString modelPath = QDir::cleanPath(
                QDir(QCoreApplication::applicationDirPath()).filePath("../3d_data/drone2.dae"));
            if (!QFile::exists(modelPath))
                return;

            auto* modelSymbol =
                new Esri::ArcGISRuntime::ModelSceneSymbol(QUrl::fromLocalFile(modelPath), 50.0);
            modelSymbol->setAnchorPosition(Esri::ArcGISRuntime::SceneSymbolAnchorPosition::Bottom);

            aircraftGraphic = new Esri::ArcGISRuntime::Graphic(
                Esri::ArcGISRuntime::Point(lon, lat, alt, Esri::ArcGISRuntime::SpatialReference::wgs84()),
                modelSymbol, nullptr);

            aircraftGraphic->attributes()->insertAttribute("ANGLE", 0.0);
            overlay->graphics()->append(aircraftGraphic);
            mapPaintWidget->m_vehicleGraphics_3d[id] = aircraftGraphic;

            if (!overlay->renderer())
            {
                auto* renderer = new Esri::ArcGISRuntime::SimpleRenderer(modelSymbol);
                Esri::ArcGISRuntime::RendererSceneProperties props;
                props.setHeadingExpression("[ANGLE]");
                overlay->setRenderer(renderer);
                renderer->setSceneProperties(props);
            }
        }

        // --- Vehicle state update ---
        qint64 now = QDateTime::currentMSecsSinceEpoch();
        auto& state = mapPaintWidget->m_vehicleState[id];

        // Initialize interpolation if first time
        if (!state.valid)
        {
            state.interpLat = lat;
            state.interpLon = lon;
            state.interpAlt = alt;
            state.valid = true;
        }

        // Update current values from UDP
        state.currLat = lat;
        state.currLon = lon;
        state.currAlt = alt;
        state.vx = vehicle.getvx(); // m/s east-west
        state.vy = vehicle.getvy(); // m/s north-south
        state.vz = vehicle.getvz(); // m/s vertical
        state.timeLast = now;

        // --- Heading (optional, if available from vehicle) ---
        // double heading = vehicle.getHeading();
        aircraftGraphic->attributes()->replaceAttribute("ANGLE", heading);

        // --- Start single global update timer once ---
        static bool timerInitialized = false;
        if (!timerInitialized)
        {
            timerInitialized = true;

            mapPaintWidget->m_updateTimer = new QTimer(mapPaintWidget);
            mapPaintWidget->m_updateTimer->setInterval(16); // ~60 FPS

            QObject::connect(mapPaintWidget->m_updateTimer, &QTimer::timeout, mapPaintWidget, [=]() {
                qint64 tNow = QDateTime::currentMSecsSinceEpoch();

                for (auto it = mapPaintWidget->m_vehicleGraphics_3d.begin();
                     it != mapPaintWidget->m_vehicleGraphics_3d.end(); ++it)
                {
                    int id = it.key();
                    Esri::ArcGISRuntime::Graphic* aircraftGraphic = it.value();
                    auto& state = mapPaintWidget->m_vehicleState[id];
                    if (!state.valid)
                        continue;

                    double dt = (tNow - state.timeLast) / 1000.0;
                    const double blend = 0.02; // smaller = smoother motion

                    // Smoothly interpolate toward latest received position
                    state.interpLat += (state.currLat - state.interpLat) * blend;
                    state.interpLon += (state.currLon - state.interpLon) * blend;
                    state.interpAlt += (state.currAlt - state.interpAlt) * blend;

                    // Extrapolate forward using velocity (per target speed)
                    double predLat = state.interpLat + (state.vy * 0.2) / 111000.0;
                    double predLon = state.interpLon + (state.vx * 0.2) /
                                                           (111000.0 * cos(state.interpLat * M_PI / 180.0));
                    double predAlt = state.interpAlt + state.vz * 0.2;

                    aircraftGraphic->setGeometry(
                        Esri::ArcGISRuntime::Point(predLon, predLat, predAlt,
                                                   Esri::ArcGISRuntime::SpatialReference::wgs84()));
                }
            });

            mapPaintWidget->m_updateTimer->start();
        }
    }
