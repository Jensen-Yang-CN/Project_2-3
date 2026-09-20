#ifndef QDEBUGSTREAM_H
#define QDEBUGSTREAM_H
#include <iostream>
#include <streambuf>
#include <QPlainTextEdit>

// 1. 自定义流缓冲类
class QDebugStream : public std::basic_streambuf<char> {
public:
    QDebugStream(std::ostream &stream, QPlainTextEdit* textedit)
        : m_stream(stream), m_old_buf(stream.rdbuf()), m_textedit(textedit)
    {
        m_stream.rdbuf(this); // 替换原来的缓冲
    }

    ~QDebugStream() {
        m_stream.rdbuf(m_old_buf); // 还原缓冲
    }

protected:
    // 当流有数据写入时调用
//    virtual int_type overflow(int_type v) override {
//        if (v != traits_type::eof()) {
//            char c = static_cast<char>(v);
//            m_buffer += c; // 使用 += 修复报错

//            // 当遇到换行符时，将缓冲区内容发送到 UI
//            if (c == '\n') {
//                QString msg = QString::fromLocal8Bit(m_buffer.c_str()).trimmed();

//                if (m_textedit) {
//                    // 确保跨线程安全
//                    QMetaObject::invokeMethod(m_textedit, [this, msg](){
//                        m_textedit->appendPlainText(msg);
//                    }, Qt::QueuedConnection);
//                }
//                m_buffer.clear();
//            }
//        }
//        return v;
//    }
    virtual int_type overflow(int_type v) override {
        if (v != traits_type::eof()) {
            char c = static_cast<char>(v);
            m_buffer += c;

            if (c == '\n') {
                // MinGW 源码通常是 UTF-8，直接用 fromUtf8 读取缓冲区
                QString msg = QString::fromUtf8(m_buffer.c_str()).trimmed();

                if (m_textedit) {
                    QMetaObject::invokeMethod(m_textedit, [=](){
                        m_textedit->appendPlainText("[System] " + msg);
                    }, Qt::QueuedConnection);
                }
                m_buffer.clear();
            }
        }
        return v;
    }

private:
    std::ostream &m_stream;
    std::streambuf *m_old_buf;
    QPlainTextEdit *m_textedit;
    std::string m_buffer;
};
#endif // QDEBUGSTREAM_H
