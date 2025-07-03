#ifndef ANALYSIS_MODULE_H
#define ANALYSIS_MODULE_H

#include <QList>
#include <QPointF>
#include <QString>
#include <QColor>
#include <vector>
#include <optional>
#include <tuple>
#include <opencv2/core/mat.hpp>
#include "utils/QtCvUtils.h"

struct AnalysisPoint {
    double x;
    double y;
    double r; // 半径，点类型可为0或无效
};

struct AnalysisTrack {
    int id;
    QList<AnalysisPoint> dataPoints;
    QColor color;
    int framesSinceUpdate = 0;
    QPointF lastCenter;
    // 新增：分析结果
    double slopeX = 0;
    double slopeY = 0;
    double angle = 0;
    double magnitude = 0;
    QString direction;
};

class AnalysisModule {
public:
    AnalysisModule();
    void clear();
    void addPoint(double x, double y, double r);
    int getPointCount() const;
    std::vector<std::tuple<double, double, double>> getPoints() const;
    std::optional<std::pair<double, double>> fitXR() const;
    std::optional<std::pair<double, double>> fitYR() const;
    QString getMoveAdvice() const;
    void addFramePoints(const std::vector<AnalysisPoint>& points);
    void update(); // 轨迹匹配与更新
    QString getSuggestion() const;
    QList<AnalysisTrack> getTracks() const;
    // 新增：获取表格数据
    QList<AnalysisTrack> getTableData() const;

private:
    QList<AnalysisTrack> m_tracks;
    int m_nextTrackId;
    QList<QColor> m_colorPalette;
    std::vector<std::tuple<double, double, double>> m_points; // x, y, r
};

#endif // ANALYSIS_MODULE_H 