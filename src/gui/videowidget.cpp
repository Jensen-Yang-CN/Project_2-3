#include "videowidget.h"

#include <QPainter>
#include <QRect>

#include <cmath>

namespace {

QRect centeredImageRect(const QSize &widgetSize, const QSize &imageSize)
{
    if (imageSize.isEmpty())
        return QRect();

    const QSize scaled = imageSize.scaled(widgetSize, Qt::KeepAspectRatio);
    return QRect((widgetSize.width() - scaled.width()) / 2,
                 (widgetSize.height() - scaled.height()) / 2,
                 scaled.width(),
                 scaled.height());
}

} // namespace

VideoWidget::VideoWidget(QWidget *parent)
    : QWidget(parent)
{
}

void VideoWidget::updateFrame(const QImage &img)
{
    QMutexLocker locker(&m_mutex);
    m_frame = img;
    m_hasFrameTimestamp = false;
    update();
}

void VideoWidget::updateFrame(const QImage &img, double timestampSec)
{
    QMutexLocker locker(&m_mutex);
    m_frame = img;
    if (timestampSec >= 0.0) {
        m_frameTimestamp = timestampSec;
        m_hasFrameTimestamp = true;
    } else {
        m_hasFrameTimestamp = false;
    }
    update();
}

void VideoWidget::updateBridgeOverlay(const QImage &overlay)
{
    QMutexLocker locker(&m_mutex);
    m_bridgeOverlay = overlay;
    m_hasOverlayTimestamp = false;
    update();
}

void VideoWidget::updateBridgeOverlay(const QImage &overlay, double timestampSec)
{
    QMutexLocker locker(&m_mutex);
    m_bridgeOverlay = overlay;
    if (timestampSec >= 0.0) {
        m_overlayTimestamp = timestampSec;
        m_hasOverlayTimestamp = true;
    } else {
        m_hasOverlayTimestamp = false;
    }
    update();
}

void VideoWidget::clearBridgeOverlay()
{
    QMutexLocker locker(&m_mutex);
    m_bridgeOverlay = QImage();
    m_hasOverlayTimestamp = false;
    update();
}

bool VideoWidget::overlayAlignedUnlocked() const
{
    if (m_bridgeOverlay.isNull())
        return false;
    if (!m_hasOverlayTimestamp)
        return true;
    if (!m_hasFrameTimestamp)
        return true;

    const double diff = m_frameTimestamp - m_overlayTimestamp;
    if (std::abs(diff) <= kTimestampToleranceSec)
        return true;
    // 桥洞检测通常晚于解码显示：画面时间戳可略大于标注时间戳
    if (diff > 0.0 && diff <= kMaxOverlayLeadSec)
        return true;
    return false;
}

void VideoWidget::paintEvent(QPaintEvent *)
{
    QImage frame;
    QImage overlay;
    bool drawOverlay = false;
    {
        QMutexLocker locker(&m_mutex);
        frame = m_frame;
        overlay = m_bridgeOverlay;
        drawOverlay = overlayAlignedUnlocked();
    }

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    if (frame.isNull())
        return;

    const QRect dest = centeredImageRect(size(), frame.size());
    const QImage scaledFrame = frame.scaled(dest.size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    painter.drawImage(dest.topLeft(), scaledFrame);

    if (drawOverlay) {
        const QImage scaledOverlay = overlay.scaled(dest.size(), Qt::IgnoreAspectRatio, Qt::FastTransformation);
        painter.drawImage(dest.topLeft(), scaledOverlay);
    }
}
