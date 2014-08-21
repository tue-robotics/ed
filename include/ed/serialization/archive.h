#ifndef ED_ARCHIVE_H_
#define ED_ARCHIVE_H_

#include <iostream>

namespace ed
{

class OArchive {

public:

    OArchive(std::ostream& stream, int version) : stream_(stream) {
        stream_.write((char*)&version, sizeof(version));
    }

    virtual ~OArchive() {}

    inline OArchive& operator<<(float f) { stream_.write((char*)&f, sizeof(f)); return *this; }

    inline OArchive& operator<<(double d) { stream_.write((char*)&d, sizeof(d)); return *this; }

    inline OArchive& operator<<(int i) { stream_.write((char*)&i, sizeof(i)); return *this; }

    inline OArchive& operator<<(std::string s) { stream_.write(s.c_str(), s.size() + 1); return *this; }

    inline std::ostream& getStream() { return stream_; }

protected:

    std::ostream& stream_;

};

class IArchive {

public:

    IArchive(std::istream& stream) : stream_(stream) {
        stream_.read((char*)&version_, sizeof(version_));
    }

    virtual ~IArchive() {}

    inline IArchive& operator>>(float& f) { stream_.read((char*)&f, sizeof(f)); return *this; }

    inline IArchive& operator>>(double& d) { stream_.read((char*)&d, sizeof(d)); return *this; }

    inline IArchive& operator>>(int& i) { stream_.read((char*)&i, sizeof(i)); return *this; }

    inline IArchive& operator>>(std::string& s) {
        s.clear();
        char c;
        while(true) {
            stream_.read(&c, 1);
            if (c == '\0') {
                break;
            }
            s += c;
        }
        return *this;
    }

    inline std::istream& getStream() { return stream_; }

    inline int getVersion() { return version_; }

protected:

    std::istream& stream_;

    int version_;

};

}

#endif
