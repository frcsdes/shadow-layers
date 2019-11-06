#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_LAYEREDFILM_H
#define PBRT_CORE_LAYEREDFILM_H

// core/layeredfilm.h*
#include "pbrt.h"
#include "film.h"

#include <ImfChannelList.h>
#include <ImfOutputFile.h>

#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace pbrt {

// _FilmLayers_ can handle several _Film_ instances. It assumes most of the
// properties are uniform among the layers, e.g. _filter_ or _croppedPixelBounds_
template<class T>
class LayeredFilm {
  public:
    template<class U>
    using Map = std::unordered_map<T, U>;

    // A set of _Tiles_ from the different layers to allow for parallel access
    struct LayeredTile : Map<std::unique_ptr<FilmTile>> {
        // A buffer to add all contributions before flushing them to the tiles
        Map<Spectrum> samples;

        // Add the gathered contributions as samples and reset the buffer
        void AddSample(const Point2f &pFilm, Float sampleWeight) {
            ProfilePhase p(Prof::LayeredFilmAddSample);

            if (this->empty()) return;
            const auto &firstTile = *this->begin()->second;

            Point2f pFilmDiscrete;
            Point2i p0;
            Point2i p1;
            firstTile.ComputeRasterBounds(pFilm, pFilmDiscrete, p0, p1);

            auto *const ifx = ALLOCA(int, p1.x - p0.x);
            auto *const ify = ALLOCA(int, p1.y - p0.y);
            firstTile.ComputeFilterSupport(pFilmDiscrete, p0, p1, ifx, ify);

            for (auto &s : samples) {
                this->at(s.first)->AddSample(s.second, p0, p1, ifx, ify);
                s.second = {};
            }
        }
    };

  public:
    LayeredFilm() = default;
    LayeredFilm(const Film &original, Map<std::string> names)
        : filename(original.filename),
          layerNames(std::move(names)) {
        // Create a _Film_ for each key
        for (const auto &name : layerNames)
            layers.emplace(name.first, original);
    }

    LayeredTile GetFilmTile(const Bounds2i &sampleBounds) {
        LayeredTile tile;
        for (auto &layer : layers) {
            tile.emplace(layer.first, layer.second.GetFilmTile(sampleBounds));
            tile.samples.emplace(layer.first, Spectrum());
        }
        return tile;
    }

    void MergeFilmTile(LayeredTile &tile) {
        for (auto &filmTile : tile)
            layers.at(filmTile.first).MergeFilmTile(std::move(filmTile.second));
    }

    void WriteImage() {
        if (layers.empty()) return;

        auto bounds = layers.begin()->second.croppedPixelBounds;
        auto width  = bounds.Diagonal().x;
        auto height = bounds.Diagonal().y;
        auto area = bounds.Area();

        Imf::Header header(width, height);
        for (const auto &name : layerNames) {
            const auto &str = name.second;
            header.channels().insert(str + "R", Imf::Channel(EXRDataName));
            header.channels().insert(str + "G", Imf::Channel(EXRDataName));
            header.channels().insert(str + "B", Imf::Channel(EXRDataName));
        }

        Imf::OutputFile file(filename.c_str(), header);
        Imf::FrameBuffer frameBuffer;

        using Buffer = std::unique_ptr<EXRDataType>;
        Map<Buffer> rBuffer;
        Map<Buffer> gBuffer;
        Map<Buffer> bBuffer;
        auto dataSize = sizeof(EXRDataType);

        for (auto &layer : layers) {
            const auto &key = layer.first;
            const auto &name = layerNames.at(key);

            rBuffer[key] = Buffer(new EXRDataType[area]);
            gBuffer[key] = Buffer(new EXRDataType[area]);
            bBuffer[key] = Buffer(new EXRDataType[area]);

            layer.second.ComputeFinalRGB(rBuffer[key].get(),
                                         gBuffer[key].get(),
                                         bBuffer[key].get());

            frameBuffer.insert(name + "R", {EXRDataName,
                (char*) rBuffer[key].get(), dataSize, dataSize * width});
            frameBuffer.insert(name + "G", {EXRDataName,
                (char*) gBuffer[key].get(), dataSize, dataSize * width});
            frameBuffer.insert(name + "B", {EXRDataName,
                (char*) bBuffer[key].get(), dataSize, dataSize * width});
        }

        file.setFrameBuffer(frameBuffer);
        file.writePixels(height);
    }

    void WriteSeparateImages() {
        if (layers.empty()) return;

        auto prefix  = filename.substr(0, filename.find_last_of("."));
        auto bounds = layers.begin()->second.croppedPixelBounds;
        auto width  = bounds.Diagonal().x;
        auto height = bounds.Diagonal().y;
        auto area = bounds.Area();

        Imf::Header header(width, height);
        header.channels().insert("R", Imf::Channel(EXRDataName));
        header.channels().insert("G", Imf::Channel(EXRDataName));
        header.channels().insert("B", Imf::Channel(EXRDataName));

        using Buffer = std::unique_ptr<EXRDataType>;
        auto rBuffer = Buffer(new EXRDataType[area]);
        auto gBuffer = Buffer(new EXRDataType[area]);
        auto bBuffer = Buffer(new EXRDataType[area]);
        auto dataSize = sizeof(EXRDataType);

        for (auto &layer : layers) {
            const auto &key = layer.first;
            const auto &name = layerNames.at(key);

            Imf::OutputFile file((prefix + name + ".exr").c_str(), header);
            Imf::FrameBuffer frameBuffer;

            layer.second.ComputeFinalRGB(rBuffer.get(),
                                         gBuffer.get(),
                                         bBuffer.get());

            frameBuffer.insert("R", {EXRDataName,
                (char*) rBuffer.get(), dataSize, dataSize * width});
            frameBuffer.insert("G", {EXRDataName,
                (char*) gBuffer.get(), dataSize, dataSize * width});
            frameBuffer.insert("B", {EXRDataName,
                (char*) bBuffer.get(), dataSize, dataSize * width});

            file.setFrameBuffer(frameBuffer);
            file.writePixels(height);
        }
    }

  private:
    std::string filename;
    Map<Film> layers;
    Map<std::string> layerNames;
};

}  // namespace pbrt

#endif  // PBRT_CORE_LAYEREDFILM_H
