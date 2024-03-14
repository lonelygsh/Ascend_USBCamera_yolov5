# Makefile
CXX = g++
CXXFLAGS = -std=c++11 `pkg-config --cflags opencv4 libavcodec libavformat libavutil libswscale`
LDFLAGS = `pkg-config --libs opencv4 libavcodec libavformat libavutil libswscale`
TARGET = orangepi_imageShow
SRC = orangepi_imageShow.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)
