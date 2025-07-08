#include <cmath>
#include <map>
#include <QtMath>
#include <QLineF>
#include <numeric>
#include "AnalysisModule.h"

// AnalysisModule 实现，负责分析点轨迹、拟合与建议生成
AnalysisModule::AnalysisModule()
    : m_nextTrackId(0)
    , m_needsFiltering(true)
{
    m_colorPalette << Qt::cyan << Qt::magenta << Qt::yellow << Qt::green << Qt::red << Qt::blue;
}

void AnalysisModule::clear()
{
    m_tracks.clear();
    m_nextTrackId = 0;
    m_points.clear();
    m_filteredPoints.clear();
    m_needsFiltering = true;
}

void AnalysisModule::addFramePoints(const std::vector<AnalysisPoint>& points)
{
    // 简单的最近邻匹配
    const double MAX_DIST = 200.0;
    for (auto& track : m_tracks) {
        track.framesSinceUpdate++;
    }
    QList<int> matchedTrackIds;
    for (const auto& pt : points) {
        QPointF center(pt.x, pt.y);
        double bestDist = MAX_DIST;
        int bestTrackId = -1;
        for (auto& track : m_tracks) {
            if (matchedTrackIds.contains(track.id)) continue;
            double dist = QLineF(center, track.lastCenter).length();
            if (dist < bestDist) {
                bestDist = dist;
                bestTrackId = track.id;
            }
        }
        if (bestTrackId != -1) {
            for (auto& track : m_tracks) {
                if (track.id == bestTrackId) {
                    track.framesSinceUpdate = 0;
                    track.lastCenter = center;
                    track.dataPoints.append(pt);
                    matchedTrackIds.append(track.id);
                    break;
                }
            }
        } else {
            AnalysisTrack newTrack;
            newTrack.id = m_nextTrackId++;
            newTrack.lastCenter = center;
            newTrack.framesSinceUpdate = 0;
            newTrack.color = m_colorPalette[newTrack.id % m_colorPalette.size()];
            newTrack.dataPoints.append(pt);
            m_tracks.append(newTrack);
        }
    }
    // 移除长时间未更新的轨迹
    const int MAX_FRAMES_UNSEEN = 100;
    for (int i = m_tracks.size() - 1; i >= 0; --i) {
        if (m_tracks[i].framesSinceUpdate > MAX_FRAMES_UNSEEN) {
            m_tracks.removeAt(i);
        }
    }
    // 计算分析结果（支持单点轨迹）
    for (auto& track : m_tracks) {
        if (track.dataPoints.size() < 1) continue;
        if (track.dataPoints.size() == 1) {
            // 单点轨迹，直接用当前点
            track.slopeX = 0;
            track.slopeY = 0;
            track.angle = 0;
            track.magnitude = 0;
            track.direction = "单点";
        } else {
            // 用QtCvUtils::linearFit进行线性拟合
            std::vector<std::pair<double, double>> xr, yr;
            for (const auto& p : track.dataPoints) {
                xr.emplace_back(p.r, p.x);
                yr.emplace_back(p.r, p.y);
            }
            auto fitX = QtCvUtils::linearFit(xr);
            auto fitY = QtCvUtils::linearFit(yr);
            track.slopeX = fitX ? fitX->first : 0;
            track.slopeY = fitY ? fitY->first : 0;
            track.angle = atan2(track.slopeY, track.slopeX) * 180.0 / M_PI;
            track.magnitude = std::sqrt(track.slopeX * track.slopeX + track.slopeY * track.slopeY);
            // 方向文本
            double angle = track.angle;
            if (angle >= -22.5 && angle < 22.5) track.direction = "右";
            else if (angle >= 22.5 && angle < 67.5) track.direction = "右下方";
            else if (angle >= 67.5 && angle < 112.5) track.direction = "下方";
            else if (angle >= 112.5 && angle < 157.5) track.direction = "左下方";
            else if (angle >= 157.5 || angle < -157.5) track.direction = "左";
            else if (angle >= -157.5 && angle < -112.5) track.direction = "左上方";
            else if (angle >= -112.5 && angle < -67.5) track.direction = "上方";
            else if (angle >= -67.5 && angle < -22.5) track.direction = "右上方";
            else track.direction = "未知";
        }
    }
}

QString AnalysisModule::getSuggestion() const
{
    QString suggestion;
    for (const auto& track : m_tracks) {
        if (track.dataPoints.size() < 1) continue;
        if (track.dataPoints.size() == 1) {
            const auto& p = track.dataPoints.first();
            suggestion += QString("轨迹%1: 当前点(%2, %3), 半径=%4\n").arg(track.id).arg(p.x, 0, 'f', 1).arg(p.y, 0, 'f', 1).arg(p.r, 0, 'f', 1);
        } else {
            suggestion += QString("轨迹%1: X-R斜率=%2, Y-R斜率=%3\n").arg(track.id).arg(track.slopeX, 0, 'f', 3).arg(track.slopeY, 0, 'f', 3);
        }
    }
    if (suggestion.isEmpty()) suggestion = "暂无足够数据进行分析";
    return suggestion;
}

QList<AnalysisTrack> AnalysisModule::getTracks() const
{
    return m_tracks;
}

QString AnalysisModule::getMoveAdvice() const
{
    auto points = filterPoints();
    // 需要足够的数据点来做有意义的拟合
    if (points.size() < 5) {
        return "<p style='color: yellow;'>数据点不足 (少于5个), 无法给出可靠建议。<br>请开始记录并确保能稳定检测到多个圆环。</p>";
    }

    auto fitX = fitXR();
    auto fitY = fitYR();

    if (!fitX || !fitY) {
        return "<p style='color: red;'>线性拟合失败，无法生成建议。</p>";
    }

    double kx = fitX->first;
    double ky = fitY->first;
    const double threshold = 0.05; // 斜率阈值，用于判断是否需要调整

    QString advice_x, advice_y;
    QString color_x, color_y;

    // --- 水平方向建议 ---
    if (std::abs(kx) < threshold) {
        advice_x = "水平方向已对准。";
        color_x = "green";
    } else if (kx > threshold) {
        advice_x = "请将光源向 <b>左</b> 移动。";
        color_x = "orange";
    } else { // kx < -threshold
        advice_x = "请将光源向 <b>右</b> 移动。";
        color_x = "orange";
    }

    // --- 垂直方向建议 ---
    if (std::abs(ky) < threshold) {
        advice_y = "垂直方向已对准。";
        color_y = "green";
    } else if (ky > threshold) {
        // y in image coordinates increases downwards
        advice_y = "请将光源向 <b>上</b> 移动。";
        color_y = "orange";
    } else { // ky < -threshold
        advice_y = "请将光源向 <b>下</b> 移动。";
        color_y = "orange";
    }
    
    QString finalAdvice = QString(
        "<p><b>对准建议:</b></p>"
        "<ul>"
        "<li><span style='color: %1;'>%2</span> (X-R斜率: %3)</li>"
        "<li><span style='color: %4;'>%5</span> (Y-R斜率: %6)</li>"
        "</ul>"
    ).arg(color_x).arg(advice_x).arg(kx, 0, 'f', 3)
     .arg(color_y).arg(advice_y).arg(ky, 0, 'f', 3);

    return finalAdvice;
}

void AnalysisModule::addPoint(double x, double y, double r) {
    m_points.emplace_back(x, y, r);
    m_needsFiltering = true;
}

int AnalysisModule::getPointCount() const {
    return static_cast<int>(m_points.size());
}

std::vector<std::tuple<double, double, double>> AnalysisModule::getPoints() const {
    return filterPoints();
}

std::vector<std::tuple<double, double, double>> AnalysisModule::filterPoints() const {
    if (!m_needsFiltering) {
        return m_filteredPoints;
    }

    if (m_points.size() < 5) { // 数据太少不进行过滤
        m_filteredPoints = m_points;
        m_needsFiltering = false;
        return m_filteredPoints;
    }

    std::vector<double> xs, ys, rs;
    for (const auto& p : m_points) {
        xs.push_back(std::get<0>(p));
        ys.push_back(std::get<1>(p));
        rs.push_back(std::get<2>(p));
    }

    auto calculate_bounds = [](std::vector<double>& data) {
        std::sort(data.begin(), data.end());
        double q1 = data[data.size() * 0.25];
        double q3 = data[data.size() * 0.75];
        double iqr = q3 - q1;
        return std::make_pair(q1 - 1.5 * iqr, q3 + 1.5 * iqr);
    };

    auto x_bounds = calculate_bounds(xs);
    auto y_bounds = calculate_bounds(ys);
    auto r_bounds = calculate_bounds(rs);

    m_filteredPoints.clear();
    for (const auto& p : m_points) {
        if (std::get<0>(p) >= x_bounds.first && std::get<0>(p) <= x_bounds.second &&
            std::get<1>(p) >= y_bounds.first && std::get<1>(p) <= y_bounds.second &&
            std::get<2>(p) >= r_bounds.first && std::get<2>(p) <= r_bounds.second)
        {
            m_filteredPoints.push_back(p);
        }
    }
    
    m_needsFiltering = false;
    return m_filteredPoints;
}

std::optional<std::pair<double, double>> AnalysisModule::fitXR() const {
    auto points = filterPoints();
    std::vector<std::pair<double, double>> data;
    for (const auto& [x, y, r] : points) {
        data.emplace_back(r, x);
    }
    return QtCvUtils::linearFit(data);
}

std::optional<std::pair<double, double>> AnalysisModule::fitYR() const {
    auto points = filterPoints();
    std::vector<std::pair<double, double>> data;
    for (const auto& [x, y, r] : points) {
        data.emplace_back(r, y);
    }
    return QtCvUtils::linearFit(data);
} 