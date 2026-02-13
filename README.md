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






    // optimised 3d trail
    else if (mapPaintWidget->MapProjection == 1) // 3D
{
    using namespace Esri::ArcGISRuntime;

    if (mapPaintWidget->Selected_id == 0)
        return;

    // ---------- persistent trail ----------
    static dtools::TrailData selectedTrail;
    static SimpleLineSymbol* trailSymbol = nullptr;
    static QDateTime lastRenderTime;

    auto* sceneView = mapPaintWidget->getMapSceneView();
    if (!sceneView || sceneView->graphicsOverlays()->isEmpty())
        return;

    auto* overlay = sceneView->graphicsOverlays()->at(0);

    // ---------- create symbol ONCE ----------
    if (!trailSymbol)
    {
        trailSymbol = new SimpleLineSymbol(
            SimpleLineSymbolStyle::Solid,
            QColor(Qt::cyan),
            3.0
        );
    }

    // ---------- selected aircraft changed ----------
    if (pre_Selected_id != mapPaintWidget->Selected_id)
    {
        pre_Selected_id = mapPaintWidget->Selected_id;

        if (selectedTrail.graphic)
            overlay->graphics()->removeOne(selectedTrail.graphic);

        selectedTrail.points.clear();
        selectedTrail.graphic = nullptr;
    }

    selectedTrail.lastUpdate = QDateTime::currentDateTimeUtc();

    // ---------- append new points ----------
    bool newPointAdded = false;

    if (mapPaintWidget->Selected_id == aircraftId && !lineStrings.isEmpty())
    {
        const LineString& ls = lineStrings.last();

        for (const dtools::geo::Pos& p : ls)
        {
            if (!p.isValid())
                continue;

            if (!selectedTrail.points.isEmpty())
            {
                const auto& last = selectedTrail.points.last();

                // fast reject (cheap)
                if (qAbs(last.getLatY() - p.getLatY()) < 1e-7 &&
                    qAbs(last.getLonX() - p.getLonX()) < 1e-7)
                    continue;

                double dist = computeDistanceMeters(last, p);

                if (dist < 1.0)
                    continue;

                // sudden teleport → reset
                if (dist > 5000.0)
                    selectedTrail.points.clear();
            }

            selectedTrail.points.append(p);
            newPointAdded = true;
        }
    }

    // ---------- limit trail ----------
    const int maxTrailPoints = 200;
    if (selectedTrail.points.size() > maxTrailPoints)
    {
        selectedTrail.points.remove(
            0,
            selectedTrail.points.size() - maxTrailPoints
        );
        newPointAdded = true;
    }

    if (selectedTrail.points.size() < 2)
        return;

    // ---------- throttle updates (10 FPS max) ----------
    QDateTime now = QDateTime::currentDateTimeUtc();
    if (!newPointAdded && lastRenderTime.msecsTo(now) < 100)
        return;

    lastRenderTime = now;

    // ---------- rebuild polyline ONLY when needed ----------
    PolylineBuilder builder(SpatialReference::wgs84());

    for (const auto& p : selectedTrail.points)
        builder.addPoint(p.getLonX(),
                         p.getLatY(),
                         p.getAltitude());

    Polyline polyline = builder.toPolyline();

    // ---------- update color without realloc ----------
    QColor color = selectedTrail.gradientFlag
                       ? QColor(Qt::yellow)
                       : QColor(Qt::cyan);

    trailSymbol->setColor(color);

    // ---------- create / update graphic ----------
    if (!selectedTrail.graphic)
    {
        selectedTrail.graphic = new Graphic(polyline, trailSymbol);
        overlay->graphics()->append(selectedTrail.graphic);
    }
    else
    {
        selectedTrail.graphic->setGeometry(polyline);
    }
}

