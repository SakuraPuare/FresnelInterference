#include <cmath>
#include <map>
#include <QtMath>
#include <QLineF>
#include <numeric>
#include "AnalysisModule.h"

// AnalysisModule 实现，负责分析点轨迹、拟合与建议生成
AnalysisModule::AnalysisModule()
    : m_nextTrackId(0)
{
    m_colorPalette << Qt::cyan << Qt::magenta << Qt::yellow << Qt::green << Qt::red << Qt::blue;
}

void AnalysisModule::clear()
{
    m_tracks.clear();
    m_nextTrackId = 0;
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
    if (m_tracks.size() < 2) return "数据不足，无法给出建议。";
    auto fitX = fitXR();
    auto fitY = fitYR();
    QString advice;
    if (fitX) {
        double kx = fitX->first;
        if (std::abs(kx) > 1e-3) {
            advice += (kx > 0) ? "光源前移，x随r增大" : "光源后移，x随r减小";
        } else {
            advice += "x与r关系不明显";
        }
    }
    advice += "\n";
    if (fitY) {
        double ky = fitY->first;
        if (std::abs(ky) > 1e-3) {
            advice += (ky > 0) ? "光源上移，y随r增大" : "光源下移，y随r减小";
        } else {
            advice += "y与r关系不明显";
        }
    }
    return advice;
}

void AnalysisModule::addPoint(double x, double y, double r) {
    m_points.emplace_back(x, y, r);
}

int AnalysisModule::getPointCount() const {
    return static_cast<int>(m_points.size());
}

std::vector<std::tuple<double, double, double>> AnalysisModule::getPoints() const {
    return m_points;
}

std::optional<std::pair<double, double>> AnalysisModule::fitXR() const {
    std::vector<std::pair<double, double>> data;
    for (const auto& [x, y, r] : m_points) {
        data.emplace_back(r, x);
    }
    return QtCvUtils::linearFit(data);
}

std::optional<std::pair<double, double>> AnalysisModule::fitYR() const {
    std::vector<std::pair<double, double>> data;
    for (const auto& [x, y, r] : m_points) {
        data.emplace_back(r, y);
    }
    return QtCvUtils::linearFit(data);
} 