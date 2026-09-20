#ifndef VIDEOWIDGET_H
#define VIDEOWIDGET_H

#include <QWidget>
#include <QImage>
#include <QMutex>

class VideoWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VideoWidget(QWidget *parent = nullptr);

public slots:
    void updateFrame(const QImage &img);
    void updateFrame(const QImage &img, double timestampSec);
    /** 桥洞检测标注层（透明 RGBA），仅在与当前帧 timestamp 对齐时绘制 */
    void updateBridgeOverlay(const QImage &overlay);
    void updateBridgeOverlay(const QImage &overlay, double timestampSec);
    void clearBridgeOverlay();

protected:
    void paintEvent(QPaintEvent *) override;

private:
    bool overlayAlignedUnlocked() const;

    QImage m_frame;
    QImage m_bridgeOverlay;
    double m_frameTimestamp = -1.0;
    double m_overlayTimestamp = -1.0;
    bool m_hasFrameTimestamp = false;
    bool m_hasOverlayTimestamp = false;
    QMutex m_mutex;

    static constexpr double kTimestampToleranceSec = 0.15;
    /** 检测晚于播放时，允许当前画面时间领先标注时间戳的最大秒数 */
    static constexpr double kMaxOverlayLeadSec = 5.0;
};

#endif // VIDEOWIDGET_H
