#ifndef ED_IO_JSON_WRITER_H_
#define ED_IO_JSON_WRITER_H_

#include "ed/io/writer.h"

#include <iostream>
#include <sstream>
#include <vector>

namespace ed::io
{

class JSONWriter : public Writer
{

public:
    explicit JSONWriter(std::ostream& out) : Writer(out) { out << "{"; }

    ~JSONWriter() override = default;

    void writeGroup(const std::string& name) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << name << "\":{";
        type_stack_.push_back('g');
        add_comma_ = false;
    }

    void endGroup() override
    {
        out_ << "}";
        if (type_stack_.empty() || type_stack_.back() != 'g')
            std::cout << "JSONWriter::endArray(): no group to close." << '\n';
        else
            type_stack_.pop_back();
        add_comma_ = true;
    }

    void writeValue(const std::string& key, float f) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":" << f;
        add_comma_ = true;
    }

    void writeValue(const std::string& key, int i) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":" << i;
        add_comma_ = true;
    }

    void writeValue(const std::string& key, const std::string& s) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":\"" << s << "\"";
        add_comma_ = true;
    }

    void writeValue(const std::string& key, double d) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":" << d;
        add_comma_ = true;
    }

    void writeValue(const std::string& key, const float* fs, std::size_t size) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":[";

        if (size > 0)
        {
            out_ << fs[0];
            for (unsigned int i = 1; i < size; ++i)
                out_ << "," << fs[i];
        }
        out_ << "]";
        add_comma_ = true;
    }

    void writeValue(const std::string& key, const int* is, std::size_t size) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":[";

        if (size > 0)
        {
            out_ << is[0];
            for (unsigned int i = 1; i < size; ++i)
                out_ << "," << is[i];
        }
        out_ << "]";
        add_comma_ = true;
    }

    void writeValue(const std::string& key, const std::string* ss, std::size_t size) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":[";

        if (size > 0)
        {
            out_ << "\"" << ss[0] << "\"";
            for (unsigned int i = 1; i < size; ++i)
                out_ << ",\"" << ss[i] << "\"";
        }
        out_ << "]";
        add_comma_ = true;
    }

    void writeArray(const std::string& key) override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "\"" << key << "\":[";
        type_stack_.push_back('a');
        add_comma_ = false;
    }

    void addArrayItem() override
    {
        if (add_comma_)
            out_ << ",";

        out_ << "{";
        type_stack_.push_back('i');
        add_comma_ = false;
    }
    void endArrayItem() override
    {
        out_ << "}";
        if (type_stack_.empty() || type_stack_.back() != 'i')
            std::cout << "JSONWriter::endArray(): no array item to close." << '\n';
        else
            type_stack_.pop_back();
        add_comma_ = true;
    }

    void endArray() override
    {
        out_ << "]";
        if (type_stack_.empty() || type_stack_.back() != 'a')
            std::cout << "JSONWriter::endArray(): no array to close." << '\n';
        else
            type_stack_.pop_back();
        add_comma_ = true;
    }

    void finish() override
    {
        // Do not pop here: endGroup()/endArrayItem()/endArray() each pop the frame they
        // close. Popping first made them either warn about a frame that was already gone,
        // or - when the next frame happened to be the same type - pop it too, losing one
        // closing token and emitting invalid JSON.
        while (!type_stack_.empty())
        {
            char const t = type_stack_.back();

            if (t == 'g')
                endGroup();
            else if (t == 'i')
                endArrayItem();
            else if (t == 'a')
                endArray();
            else
                type_stack_.pop_back(); // unreachable; guards against an endless loop
        }
        out_ << "}";
    }

private:
    bool add_comma_{false};
    std::vector<char> type_stack_;
};

} // namespace ed::io

#endif
