#define PY_SSIZE_T_CLEAN
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

// On MinGW, Python's Windows pyconfig remaps ``hypot`` before libstdc++ has
// declared it.  Loading cmath first avoids that toolchain-only conflict and
// is a no-op on the Raspberry Pi/GCC build.
#include <cmath>
#ifdef __MINGW32__
namespace std {
inline double _hypot(double x, double y) {
    return std::hypot(x, y);
}
}  // namespace std
#endif
#include <Python.h>
#ifdef __MINGW32__
#undef hypot
#endif
#include <numpy/arrayobject.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

struct Point2d {
    double x;
    double y;
};

double cross(const Point2d& a, const Point2d& b, const Point2d& point);

bool point_less(const Point2d& first, const Point2d& second) {
    return first.x < second.x
        || (first.x == second.x && first.y < second.y);
}

std::vector<Point2d> convex_hull(std::vector<Point2d> points) {
    if (points.size() <= 1) {
        return points;
    }
    std::sort(points.begin(), points.end(), point_less);
    points.erase(
        std::unique(
            points.begin(),
            points.end(),
            [](const Point2d& first, const Point2d& second) {
                return first.x == second.x && first.y == second.y;
            }
        ),
        points.end()
    );
    if (points.size() <= 2) {
        return points;
    }
    std::vector<Point2d> hull;
    hull.reserve(points.size() * 2);
    for (const Point2d& point : points) {
        while (
            hull.size() >= 2
            && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0
        ) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const std::size_t lower_size = hull.size();
    for (std::size_t reverse = points.size() - 1; reverse > 0; --reverse) {
        const Point2d& point = points[reverse - 1];
        while (
            hull.size() > lower_size
            && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0
        ) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    hull.pop_back();
    return hull;
}

std::array<double, 2> minimum_bounding_sides(
    const std::vector<Point2d>& points
) {
    const std::vector<Point2d> hull = convex_hull(points);
    if (hull.size() < 2) {
        return {0.0, 0.0};
    }
    double best_area = std::numeric_limits<double>::infinity();
    double best_width = 0.0;
    double best_height = 0.0;
    for (std::size_t edge = 0; edge < hull.size(); ++edge) {
        const Point2d& start = hull[edge];
        const Point2d& end = hull[(edge + 1) % hull.size()];
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-12) {
            continue;
        }
        const double ux = dx / length;
        const double uy = dy / length;
        const double vx = -uy;
        const double vy = ux;
        double minimum_u = std::numeric_limits<double>::infinity();
        double maximum_u = -minimum_u;
        double minimum_v = std::numeric_limits<double>::infinity();
        double maximum_v = -minimum_v;
        for (const Point2d& point : hull) {
            const double projection_u = point.x * ux + point.y * uy;
            const double projection_v = point.x * vx + point.y * vy;
            minimum_u = std::min(minimum_u, projection_u);
            maximum_u = std::max(maximum_u, projection_u);
            minimum_v = std::min(minimum_v, projection_v);
            maximum_v = std::max(maximum_v, projection_v);
        }
        const double width = maximum_u - minimum_u;
        const double height = maximum_v - minimum_v;
        const double area = width * height;
        if (area < best_area) {
            best_area = area;
            best_width = width;
            best_height = height;
        }
    }
    return {
        std::min(best_width, best_height),
        std::max(best_width, best_height),
    };
}

double cross(const Point2d& a, const Point2d& b, const Point2d& point) {
    return (b.x - a.x) * (point.y - a.y)
        - (b.y - a.y) * (point.x - a.x);
}

double signed_polygon_area(const std::vector<Point2d>& polygon) {
    double twice_area = 0.0;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const Point2d& first = polygon[index];
        const Point2d& second = polygon[(index + 1) % polygon.size()];
        twice_area += first.x * second.y - first.y * second.x;
    }
    return 0.5 * twice_area;
}

double convex_overlap_area_impl(
    const float* first,
    int first_count,
    const float* second,
    int second_count
) {
    if (first_count < 3 || second_count < 3) {
        return 0.0;
    }
    std::vector<Point2d> output;
    output.reserve(static_cast<std::size_t>(first_count + second_count));
    for (int index = 0; index < first_count; ++index) {
        output.push_back({
            static_cast<double>(first[index * 2]),
            static_cast<double>(first[index * 2 + 1]),
        });
    }

    std::vector<Point2d> clip;
    clip.reserve(static_cast<std::size_t>(second_count));
    for (int index = 0; index < second_count; ++index) {
        clip.push_back({
            static_cast<double>(second[index * 2]),
            static_cast<double>(second[index * 2 + 1]),
        });
    }
    const double orientation = signed_polygon_area(clip) >= 0.0 ? 1.0 : -1.0;
    constexpr double epsilon = 1e-9;

    for (int edge_index = 0; edge_index < second_count; ++edge_index) {
        if (output.empty()) {
            return 0.0;
        }
        const Point2d clip_start = clip[edge_index];
        const Point2d clip_end = clip[(edge_index + 1) % second_count];
        std::vector<Point2d> input;
        input.swap(output);
        output.clear();
        output.reserve(input.size() + 1);

        Point2d start = input.back();
        double start_side = orientation * cross(clip_start, clip_end, start);
        bool start_inside = start_side >= -epsilon;
        for (const Point2d& end : input) {
            const double end_side = orientation * cross(
                clip_start,
                clip_end,
                end
            );
            const bool end_inside = end_side >= -epsilon;
            if (start_inside != end_inside) {
                const double denominator = start_side - end_side;
                if (std::abs(denominator) > epsilon) {
                    const double ratio = start_side / denominator;
                    output.push_back({
                        start.x + ratio * (end.x - start.x),
                        start.y + ratio * (end.y - start.y),
                    });
                }
            }
            if (end_inside) {
                output.push_back(end);
            }
            start = end;
            start_side = end_side;
            start_inside = end_inside;
        }
    }
    return std::abs(signed_polygon_area(output));
}

double best_shifted_overlap_impl(
    const npy_bool* first,
    const npy_bool* second,
    npy_intp height,
    npy_intp width,
    int maximum_shift
) {
    std::size_t first_count = 0;
    const std::size_t pixel_count = static_cast<std::size_t>(height * width);
    for (std::size_t index = 0; index < pixel_count; ++index) {
        first_count += static_cast<unsigned char>(first[index]);
    }

    double best = 0.0;
    for (int shift_y = -maximum_shift; shift_y <= maximum_shift; ++shift_y) {
        const npy_intp first_y0 = std::max(0, shift_y);
        const npy_intp second_y0 = std::max(0, -shift_y);
        const npy_intp overlap_height = height - std::abs(shift_y);
        if (overlap_height <= 0) {
            continue;
        }
        for (int shift_x = -maximum_shift; shift_x <= maximum_shift; ++shift_x) {
            const npy_intp first_x0 = std::max(0, shift_x);
            const npy_intp second_x0 = std::max(0, -shift_x);
            const npy_intp overlap_width = width - std::abs(shift_x);
            if (overlap_width <= 0) {
                continue;
            }

            std::size_t second_count = 0;
            std::size_t intersection = 0;
            for (npy_intp y = 0; y < overlap_height; ++y) {
                const npy_bool* first_row = first
                    + (first_y0 + y) * width + first_x0;
                const npy_bool* second_row = second
                    + (second_y0 + y) * width + second_x0;
                for (npy_intp x = 0; x < overlap_width; ++x) {
                    const unsigned char second_set =
                        static_cast<unsigned char>(second_row[x]);
                    second_count += second_set;
                    intersection += second_set
                        & static_cast<unsigned char>(first_row[x]);
                }
            }
            const std::size_t denominator = first_count + second_count;
            if (denominator != 0) {
                best = std::max(
                    best,
                    2.0 * static_cast<double>(intersection)
                        / static_cast<double>(denominator)
                );
            }
        }
    }
    return best;
}

bool check_pair_shapes(PyArrayObject* first, PyArrayObject* second, int ndim) {
    if (PyArray_NDIM(first) != ndim || PyArray_NDIM(second) != ndim) {
        PyErr_Format(
            PyExc_ValueError,
            "expected two equally sized %d-D masks",
            ndim
        );
        return false;
    }
    for (int axis = 0; axis < ndim; ++axis) {
        if (PyArray_DIM(first, axis) != PyArray_DIM(second, axis)) {
            PyErr_SetString(PyExc_ValueError, "mask shapes must match");
            return false;
        }
    }
    return true;
}

PyObject* best_shifted_overlap(PyObject*, PyObject* args, PyObject* kwargs) {
    PyObject* first_object = nullptr;
    PyObject* second_object = nullptr;
    int maximum_shift = 4;
    static const char* keywords[] = {
        "first", "second", "maximum_shift_px", nullptr
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OO|i",
            const_cast<char**>(keywords),
            &first_object,
            &second_object,
            &maximum_shift)) {
        return nullptr;
    }
    if (maximum_shift < 0) {
        PyErr_SetString(PyExc_ValueError, "maximum_shift_px must be non-negative");
        return nullptr;
    }

    PyArrayObject* first = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            first_object,
            NPY_BOOL,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* second = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            second_object,
            NPY_BOOL,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    if (first == nullptr || second == nullptr) {
        Py_XDECREF(first);
        Py_XDECREF(second);
        return nullptr;
    }
    if (!check_pair_shapes(first, second, 2)) {
        Py_DECREF(first);
        Py_DECREF(second);
        return nullptr;
    }

    const npy_intp height = PyArray_DIM(first, 0);
    const npy_intp width = PyArray_DIM(first, 1);
    double result = 0.0;
    Py_BEGIN_ALLOW_THREADS
    result = best_shifted_overlap_impl(
        static_cast<const npy_bool*>(PyArray_DATA(first)),
        static_cast<const npy_bool*>(PyArray_DATA(second)),
        height,
        width,
        maximum_shift
    );
    Py_END_ALLOW_THREADS
    Py_DECREF(first);
    Py_DECREF(second);
    return PyFloat_FromDouble(result);
}

PyObject* batch_best_shifted_overlap(
    PyObject*,
    PyObject* args,
    PyObject* kwargs
) {
    PyObject* first_object = nullptr;
    PyObject* second_object = nullptr;
    int maximum_shift = 4;
    int workers = 1;
    static const char* keywords[] = {
        "first", "second", "maximum_shift_px", "workers", nullptr
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OO|ii",
            const_cast<char**>(keywords),
            &first_object,
            &second_object,
            &maximum_shift,
            &workers)) {
        return nullptr;
    }
    if (maximum_shift < 0 || workers < 1) {
        PyErr_SetString(
            PyExc_ValueError,
            "maximum_shift_px must be non-negative and workers must be positive"
        );
        return nullptr;
    }

    PyArrayObject* first = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            first_object,
            NPY_BOOL,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* second = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            second_object,
            NPY_BOOL,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    if (first == nullptr || second == nullptr) {
        Py_XDECREF(first);
        Py_XDECREF(second);
        return nullptr;
    }
    if (!check_pair_shapes(first, second, 3)) {
        Py_DECREF(first);
        Py_DECREF(second);
        return nullptr;
    }

    const npy_intp batch = PyArray_DIM(first, 0);
    const npy_intp height = PyArray_DIM(first, 1);
    const npy_intp width = PyArray_DIM(first, 2);
    npy_intp output_shape[] = {batch};
    PyArrayObject* output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(1, output_shape, NPY_DOUBLE)
    );
    if (output == nullptr) {
        Py_DECREF(first);
        Py_DECREF(second);
        return nullptr;
    }

    const npy_bool* first_data = static_cast<const npy_bool*>(
        PyArray_DATA(first)
    );
    const npy_bool* second_data = static_cast<const npy_bool*>(
        PyArray_DATA(second)
    );
    double* output_data = static_cast<double*>(PyArray_DATA(output));
    const npy_intp mask_stride = height * width;

    Py_BEGIN_ALLOW_THREADS
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers)
#endif
    for (npy_intp index = 0; index < batch; ++index) {
        output_data[index] = best_shifted_overlap_impl(
            first_data + index * mask_stride,
            second_data + index * mask_stride,
            height,
            width,
            maximum_shift
        );
    }
    Py_END_ALLOW_THREADS

    Py_DECREF(first);
    Py_DECREF(second);
    return reinterpret_cast<PyObject*>(output);
}

PyObject* outer_corner_metrics(PyObject*, PyObject* args, PyObject* kwargs) {
    PyObject* outline_object = nullptr;
    PyObject* box_object = nullptr;
    double probe_distance = 6.0;
    static const char* keywords[] = {
        "outline", "box", "probe_distance_mm", nullptr
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OO|d",
            const_cast<char**>(keywords),
            &outline_object,
            &box_object,
            &probe_distance)) {
        return nullptr;
    }
    if (probe_distance < 0.0) {
        PyErr_SetString(PyExc_ValueError, "probe_distance_mm must be non-negative");
        return nullptr;
    }

    PyArrayObject* outline = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            outline_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* box = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            box_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    if (outline == nullptr || box == nullptr) {
        Py_XDECREF(outline);
        Py_XDECREF(box);
        return nullptr;
    }
    const bool shapes_valid =
        PyArray_NDIM(outline) == 2
        && PyArray_DIM(outline, 1) == 2
        && PyArray_NDIM(box) == 2
        && PyArray_DIM(box, 0) == 4
        && PyArray_DIM(box, 1) == 2;
    if (!shapes_valid) {
        Py_DECREF(outline);
        Py_DECREF(box);
        PyErr_SetString(
            PyExc_ValueError,
            "outline must be Nx2 and box must be 4x2"
        );
        return nullptr;
    }

    const npy_intp outline_count = PyArray_DIM(outline, 0);
    if (outline_count < 4) {
        Py_DECREF(outline);
        Py_DECREF(box);
        return Py_BuildValue("Ndd", PyList_New(0), Py_HUGE_VAL, Py_HUGE_VAL);
    }
    const float* outline_data = static_cast<const float*>(
        PyArray_DATA(outline)
    );
    const float* box_data = static_cast<const float*>(PyArray_DATA(box));
    std::array<double, 4> angles{};
    std::array<double, 4> offsets{};
    bool valid = true;
    const float probe_squared = static_cast<float>(
        probe_distance * probe_distance
    );

    Py_BEGIN_ALLOW_THREADS
    for (int corner = 0; corner < 4 && valid; ++corner) {
        const float box_x = box_data[corner * 2];
        const float box_y = box_data[corner * 2 + 1];
        npy_intp corner_index = 0;
        float best_distance_squared = std::numeric_limits<float>::infinity();
        for (npy_intp index = 0; index < outline_count; ++index) {
            const float dx = outline_data[index * 2] - box_x;
            const float dy = outline_data[index * 2 + 1] - box_y;
            const float distance_squared = dx * dx + dy * dy;
            if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                corner_index = index;
            }
        }
        offsets[corner] = std::sqrt(
            static_cast<double>(best_distance_squared)
        );

        const float anchor_x = outline_data[corner_index * 2];
        const float anchor_y = outline_data[corner_index * 2 + 1];
        std::array<float, 2> vectors[2]{};
        for (int direction_id = 0; direction_id < 2; ++direction_id) {
            const int direction = direction_id == 0 ? -1 : 1;
            bool found = false;
            for (npy_intp step = 1; step < outline_count; ++step) {
                npy_intp index = (
                    corner_index + direction * step
                ) % outline_count;
                if (index < 0) {
                    index += outline_count;
                }
                const float dx = outline_data[index * 2] - anchor_x;
                const float dy = outline_data[index * 2 + 1] - anchor_y;
                if (dx * dx + dy * dy >= probe_squared) {
                    vectors[direction_id] = {dx, dy};
                    found = true;
                    break;
                }
            }
            if (!found) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            break;
        }

        const float first_squared =
            vectors[0][0] * vectors[0][0]
            + vectors[0][1] * vectors[0][1];
        const float second_squared =
            vectors[1][0] * vectors[1][0]
            + vectors[1][1] * vectors[1][1];
        const double denominator = std::max(
            1e-8,
            std::sqrt(
                static_cast<double>(first_squared)
                * static_cast<double>(second_squared)
            )
        );
        const float dot =
            vectors[0][0] * vectors[1][0]
            + vectors[0][1] * vectors[1][1];
        const double cosine = std::max(
            -1.0,
            std::min(1.0, static_cast<double>(dot) / denominator)
        );
        angles[corner] = std::acos(cosine) * 180.0 / 3.14159265358979323846;
    }
    Py_END_ALLOW_THREADS

    Py_DECREF(outline);
    Py_DECREF(box);
    if (!valid) {
        return Py_BuildValue("Ndd", PyList_New(0), Py_HUGE_VAL, Py_HUGE_VAL);
    }

    PyObject* angle_list = PyList_New(4);
    if (angle_list == nullptr) {
        return nullptr;
    }
    double maximum_error = 0.0;
    double maximum_offset = 0.0;
    for (int index = 0; index < 4; ++index) {
        PyList_SET_ITEM(angle_list, index, PyFloat_FromDouble(angles[index]));
        maximum_error = std::max(
            maximum_error,
            std::abs(angles[index] - 90.0)
        );
        maximum_offset = std::max(maximum_offset, offsets[index]);
    }
    PyObject* result = PyTuple_New(3);
    if (result == nullptr) {
        Py_DECREF(angle_list);
        return nullptr;
    }
    PyTuple_SET_ITEM(result, 0, angle_list);
    PyTuple_SET_ITEM(result, 1, PyFloat_FromDouble(maximum_error));
    PyTuple_SET_ITEM(result, 2, PyFloat_FromDouble(maximum_offset));
    return result;
}

PyObject* batch_convex_overlap_areas(
    PyObject*,
    PyObject* args,
    PyObject* kwargs
) {
    PyObject* first_object = nullptr;
    PyObject* first_counts_object = nullptr;
    PyObject* second_object = nullptr;
    PyObject* second_counts_object = nullptr;
    int workers = 1;
    static const char* keywords[] = {
        "first",
        "first_counts",
        "second",
        "second_counts",
        "workers",
        nullptr,
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OOOO|i",
            const_cast<char**>(keywords),
            &first_object,
            &first_counts_object,
            &second_object,
            &second_counts_object,
            &workers)) {
        return nullptr;
    }
    if (workers < 1) {
        PyErr_SetString(PyExc_ValueError, "workers must be positive");
        return nullptr;
    }

    PyArrayObject* first = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            first_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* second = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            second_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* first_counts = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            first_counts_object,
            NPY_INT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* second_counts = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            second_counts_object,
            NPY_INT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    if (
        first == nullptr
        || second == nullptr
        || first_counts == nullptr
        || second_counts == nullptr
    ) {
        Py_XDECREF(first);
        Py_XDECREF(second);
        Py_XDECREF(first_counts);
        Py_XDECREF(second_counts);
        return nullptr;
    }

    const bool shapes_valid =
        PyArray_NDIM(first) == 3
        && PyArray_DIM(first, 2) == 2
        && PyArray_NDIM(second) == 3
        && PyArray_DIM(second, 2) == 2
        && PyArray_DIM(first, 0) == PyArray_DIM(second, 0)
        && PyArray_NDIM(first_counts) == 1
        && PyArray_NDIM(second_counts) == 1
        && PyArray_DIM(first_counts, 0) == PyArray_DIM(first, 0)
        && PyArray_DIM(second_counts, 0) == PyArray_DIM(first, 0);
    if (!shapes_valid) {
        Py_DECREF(first);
        Py_DECREF(second);
        Py_DECREF(first_counts);
        Py_DECREF(second_counts);
        PyErr_SetString(
            PyExc_ValueError,
            "polygons must be NxVx2 and count arrays must have length N"
        );
        return nullptr;
    }

    const npy_intp batch = PyArray_DIM(first, 0);
    const npy_intp first_max_vertices = PyArray_DIM(first, 1);
    const npy_intp second_max_vertices = PyArray_DIM(second, 1);
    const float* first_data = static_cast<const float*>(PyArray_DATA(first));
    const float* second_data = static_cast<const float*>(PyArray_DATA(second));
    const int* first_count_data = static_cast<const int*>(
        PyArray_DATA(first_counts)
    );
    const int* second_count_data = static_cast<const int*>(
        PyArray_DATA(second_counts)
    );
    for (npy_intp index = 0; index < batch; ++index) {
        if (
            first_count_data[index] < 0
            || first_count_data[index] > first_max_vertices
            || second_count_data[index] < 0
            || second_count_data[index] > second_max_vertices
        ) {
            Py_DECREF(first);
            Py_DECREF(second);
            Py_DECREF(first_counts);
            Py_DECREF(second_counts);
            PyErr_SetString(PyExc_ValueError, "invalid polygon vertex count");
            return nullptr;
        }
    }

    npy_intp output_shape[] = {batch};
    PyArrayObject* output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(1, output_shape, NPY_DOUBLE)
    );
    if (output == nullptr) {
        Py_DECREF(first);
        Py_DECREF(second);
        Py_DECREF(first_counts);
        Py_DECREF(second_counts);
        return nullptr;
    }
    double* output_data = static_cast<double*>(PyArray_DATA(output));
    const npy_intp first_stride = first_max_vertices * 2;
    const npy_intp second_stride = second_max_vertices * 2;

    Py_BEGIN_ALLOW_THREADS
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers)
#endif
    for (npy_intp index = 0; index < batch; ++index) {
        output_data[index] = convex_overlap_area_impl(
            first_data + index * first_stride,
            first_count_data[index],
            second_data + index * second_stride,
            second_count_data[index]
        );
    }
    Py_END_ALLOW_THREADS

    Py_DECREF(first);
    Py_DECREF(second);
    Py_DECREF(first_counts);
    Py_DECREF(second_counts);
    return reinterpret_cast<PyObject*>(output);
}

PyObject* batch_edge_alignment_world(
    PyObject*,
    PyObject* args,
    PyObject* kwargs
) {
    PyObject* polygons_object = nullptr;
    PyObject* counts_object = nullptr;
    PyObject* moving_ids_object = nullptr;
    PyObject* moving_edges_object = nullptr;
    PyObject* fixed_polygons_object = nullptr;
    PyObject* fixed_counts_object = nullptr;
    PyObject* fixed_edges_object = nullptr;
    PyObject* kinds_object = nullptr;
    PyObject* offsets_object = nullptr;
    int workers = 1;
    static const char* keywords[] = {
        "polygons",
        "counts",
        "moving_ids",
        "moving_edges",
        "fixed_polygons",
        "fixed_counts",
        "fixed_edges",
        "kinds",
        "offsets",
        "workers",
        nullptr,
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OOOOOOOOO|i",
            const_cast<char**>(keywords),
            &polygons_object,
            &counts_object,
            &moving_ids_object,
            &moving_edges_object,
            &fixed_polygons_object,
            &fixed_counts_object,
            &fixed_edges_object,
            &kinds_object,
            &offsets_object,
            &workers)) {
        return nullptr;
    }
    if (workers < 1) {
        PyErr_SetString(PyExc_ValueError, "workers must be positive");
        return nullptr;
    }

#define LOAD_ARRAY(name, object, type) \
    PyArrayObject* name = reinterpret_cast<PyArrayObject*>( \
        PyArray_FROM_OTF( \
            object, type, NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST \
        ) \
    )
    LOAD_ARRAY(polygons, polygons_object, NPY_FLOAT32);
    LOAD_ARRAY(counts, counts_object, NPY_INT32);
    LOAD_ARRAY(moving_ids, moving_ids_object, NPY_INT32);
    LOAD_ARRAY(moving_edges, moving_edges_object, NPY_INT32);
    LOAD_ARRAY(fixed_polygons, fixed_polygons_object, NPY_FLOAT32);
    LOAD_ARRAY(fixed_counts, fixed_counts_object, NPY_INT32);
    LOAD_ARRAY(fixed_edges, fixed_edges_object, NPY_INT32);
    LOAD_ARRAY(kinds, kinds_object, NPY_INT32);
    LOAD_ARRAY(offsets, offsets_object, NPY_DOUBLE);
#undef LOAD_ARRAY
    if (
        polygons == nullptr
        || counts == nullptr
        || moving_ids == nullptr
        || moving_edges == nullptr
        || fixed_polygons == nullptr
        || fixed_counts == nullptr
        || fixed_edges == nullptr
        || kinds == nullptr
        || offsets == nullptr
    ) {
        Py_XDECREF(polygons);
        Py_XDECREF(counts);
        Py_XDECREF(moving_ids);
        Py_XDECREF(moving_edges);
        Py_XDECREF(fixed_polygons);
        Py_XDECREF(fixed_counts);
        Py_XDECREF(fixed_edges);
        Py_XDECREF(kinds);
        Py_XDECREF(offsets);
        return nullptr;
    }

    const npy_intp piece_count = PyArray_DIM(polygons, 0);
    const npy_intp maximum_vertices = PyArray_DIM(polygons, 1);
    const npy_intp job_count = PyArray_DIM(moving_ids, 0);
    const npy_intp fixed_maximum_vertices = PyArray_DIM(fixed_polygons, 1);
    const bool shapes_valid =
        PyArray_NDIM(polygons) == 3
        && PyArray_DIM(polygons, 2) == 2
        && PyArray_NDIM(counts) == 1
        && PyArray_DIM(counts, 0) == piece_count
        && PyArray_NDIM(moving_ids) == 1
        && PyArray_NDIM(moving_edges) == 1
        && PyArray_DIM(moving_edges, 0) == job_count
        && PyArray_NDIM(fixed_polygons) == 3
        && PyArray_DIM(fixed_polygons, 0) == job_count
        && PyArray_DIM(fixed_polygons, 2) == 2
        && PyArray_NDIM(fixed_counts) == 1
        && PyArray_DIM(fixed_counts, 0) == job_count
        && PyArray_NDIM(fixed_edges) == 1
        && PyArray_DIM(fixed_edges, 0) == job_count
        && PyArray_NDIM(kinds) == 1
        && PyArray_DIM(kinds, 0) == job_count
        && PyArray_NDIM(offsets) == 1
        && PyArray_DIM(offsets, 0) == job_count;
    if (!shapes_valid) {
        Py_DECREF(polygons);
        Py_DECREF(counts);
        Py_DECREF(moving_ids);
        Py_DECREF(moving_edges);
        Py_DECREF(fixed_polygons);
        Py_DECREF(fixed_counts);
        Py_DECREF(fixed_edges);
        Py_DECREF(kinds);
        Py_DECREF(offsets);
        PyErr_SetString(PyExc_ValueError, "invalid edge alignment array shapes");
        return nullptr;
    }

    const int* count_data = static_cast<const int*>(PyArray_DATA(counts));
    const int* moving_id_data = static_cast<const int*>(
        PyArray_DATA(moving_ids)
    );
    const int* moving_edge_data = static_cast<const int*>(
        PyArray_DATA(moving_edges)
    );
    const int* fixed_count_data = static_cast<const int*>(
        PyArray_DATA(fixed_counts)
    );
    const int* fixed_edge_data = static_cast<const int*>(
        PyArray_DATA(fixed_edges)
    );
    const int* kind_data = static_cast<const int*>(PyArray_DATA(kinds));
    for (npy_intp job = 0; job < job_count; ++job) {
        const int moving_id = moving_id_data[job];
        if (
            moving_id < 0
            || moving_id >= piece_count
            || count_data[moving_id] < 2
            || count_data[moving_id] > maximum_vertices
            || moving_edge_data[job] < 0
            || moving_edge_data[job] >= count_data[moving_id]
            || fixed_count_data[job] < 2
            || fixed_count_data[job] > fixed_maximum_vertices
            || fixed_edge_data[job] < 0
            || fixed_edge_data[job] >= fixed_count_data[job]
            || kind_data[job] < 0
            || kind_data[job] > 3
        ) {
            Py_DECREF(polygons);
            Py_DECREF(counts);
            Py_DECREF(moving_ids);
            Py_DECREF(moving_edges);
            Py_DECREF(fixed_polygons);
            Py_DECREF(fixed_counts);
            Py_DECREF(fixed_edges);
            Py_DECREF(kinds);
            Py_DECREF(offsets);
            PyErr_SetString(PyExc_ValueError, "invalid edge alignment job");
            return nullptr;
        }
    }

    npy_intp rotation_shape[] = {job_count, 2, 2};
    npy_intp translation_shape[] = {job_count, 2};
    npy_intp world_shape[] = {job_count, maximum_vertices, 2};
    PyArrayObject* rotation_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(3, rotation_shape, NPY_FLOAT32)
    );
    PyArrayObject* translation_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(2, translation_shape, NPY_FLOAT32)
    );
    PyArrayObject* world_output = reinterpret_cast<PyArrayObject*>(
        PyArray_ZEROS(3, world_shape, NPY_FLOAT32, 0)
    );
    if (
        rotation_output == nullptr
        || translation_output == nullptr
        || world_output == nullptr
    ) {
        Py_XDECREF(rotation_output);
        Py_XDECREF(translation_output);
        Py_XDECREF(world_output);
        Py_DECREF(polygons);
        Py_DECREF(counts);
        Py_DECREF(moving_ids);
        Py_DECREF(moving_edges);
        Py_DECREF(fixed_polygons);
        Py_DECREF(fixed_counts);
        Py_DECREF(fixed_edges);
        Py_DECREF(kinds);
        Py_DECREF(offsets);
        return nullptr;
    }

    const float* polygon_data = static_cast<const float*>(
        PyArray_DATA(polygons)
    );
    const float* fixed_data = static_cast<const float*>(
        PyArray_DATA(fixed_polygons)
    );
    const double* offset_data = static_cast<const double*>(
        PyArray_DATA(offsets)
    );
    float* rotation_data = static_cast<float*>(PyArray_DATA(rotation_output));
    float* translation_data = static_cast<float*>(
        PyArray_DATA(translation_output)
    );
    float* world_data = static_cast<float*>(PyArray_DATA(world_output));
    const npy_intp polygon_stride = maximum_vertices * 2;
    const npy_intp fixed_stride = fixed_maximum_vertices * 2;

    Py_BEGIN_ALLOW_THREADS
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers)
#endif
    for (npy_intp job = 0; job < job_count; ++job) {
        const int moving_id = moving_id_data[job];
        const int moving_count = count_data[moving_id];
        const int moving_edge = moving_edge_data[job];
        const int fixed_count = fixed_count_data[job];
        const int fixed_edge = fixed_edge_data[job];
        const float* moving = polygon_data + moving_id * polygon_stride;
        const float* fixed = fixed_data + job * fixed_stride;
        const int moving_next = (moving_edge + 1) % moving_count;
        const int fixed_next = (fixed_edge + 1) % fixed_count;
        const double moving_start_x = moving[moving_edge * 2];
        const double moving_start_y = moving[moving_edge * 2 + 1];
        const double moving_end_x = moving[moving_next * 2];
        const double moving_end_y = moving[moving_next * 2 + 1];
        const double fixed_start_x = fixed[fixed_edge * 2];
        const double fixed_start_y = fixed[fixed_edge * 2 + 1];
        const double fixed_end_x = fixed[fixed_next * 2];
        const double fixed_end_y = fixed[fixed_next * 2 + 1];
        const double moving_dx = moving_end_x - moving_start_x;
        const double moving_dy = moving_end_y - moving_start_y;
        const double fixed_dx = fixed_start_x - fixed_end_x;
        const double fixed_dy = fixed_start_y - fixed_end_y;
        const double angle = std::atan2(fixed_dy, fixed_dx)
            - std::atan2(moving_dy, moving_dx);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        const auto rotated = [cosine, sine](double x, double y) {
            return Point2d{
                cosine * x - sine * y,
                sine * x + cosine * y,
            };
        };
        const Point2d rotated_start = rotated(moving_start_x, moving_start_y);
        const Point2d rotated_end = rotated(moving_end_x, moving_end_y);
        double translation_x = 0.0;
        double translation_y = 0.0;
        const int kind = kind_data[job];
        if (kind == 0) {
            const Point2d rotated_midpoint = rotated(
                (moving_start_x + moving_end_x) * 0.5,
                (moving_start_y + moving_end_y) * 0.5
            );
            translation_x = (fixed_start_x + fixed_end_x) * 0.5
                - rotated_midpoint.x;
            translation_y = (fixed_start_y + fixed_end_y) * 0.5
                - rotated_midpoint.y;
        } else if (kind == 1) {
            translation_x = fixed_start_x - rotated_end.x;
            translation_y = fixed_start_y - rotated_end.y;
        } else if (kind == 2) {
            translation_x = fixed_end_x - rotated_start.x;
            translation_y = fixed_end_y - rotated_start.y;
        } else {
            const double moving_length = std::hypot(moving_dx, moving_dy);
            const double fixed_length = std::hypot(fixed_dx, fixed_dy);
            const double offset = offset_data[job];
            if (fixed_length >= moving_length) {
                const double target_x = fixed_end_x
                    + fixed_dx / fixed_length * offset;
                const double target_y = fixed_end_y
                    + fixed_dy / fixed_length * offset;
                translation_x = target_x - rotated_start.x;
                translation_y = target_y - rotated_start.y;
            } else {
                const double sub_start_x = moving_start_x
                    + moving_dx / moving_length * offset;
                const double sub_start_y = moving_start_y
                    + moving_dy / moving_length * offset;
                const Point2d rotated_sub_start = rotated(
                    sub_start_x,
                    sub_start_y
                );
                translation_x = fixed_end_x - rotated_sub_start.x;
                translation_y = fixed_end_y - rotated_sub_start.y;
            }
        }

        float* output_rotation = rotation_data + job * 4;
        output_rotation[0] = static_cast<float>(cosine);
        output_rotation[1] = static_cast<float>(-sine);
        output_rotation[2] = static_cast<float>(sine);
        output_rotation[3] = static_cast<float>(cosine);
        translation_data[job * 2] = static_cast<float>(translation_x);
        translation_data[job * 2 + 1] = static_cast<float>(translation_y);
        float* output_world = world_data + job * polygon_stride;
        for (int point = 0; point < moving_count; ++point) {
            const Point2d transformed = rotated(
                moving[point * 2],
                moving[point * 2 + 1]
            );
            output_world[point * 2] = static_cast<float>(
                transformed.x + translation_x
            );
            output_world[point * 2 + 1] = static_cast<float>(
                transformed.y + translation_y
            );
        }
    }
    Py_END_ALLOW_THREADS

    Py_DECREF(polygons);
    Py_DECREF(counts);
    Py_DECREF(moving_ids);
    Py_DECREF(moving_edges);
    Py_DECREF(fixed_polygons);
    Py_DECREF(fixed_counts);
    Py_DECREF(fixed_edges);
    Py_DECREF(kinds);
    Py_DECREF(offsets);
    PyObject* result = PyTuple_New(3);
    if (result == nullptr) {
        Py_DECREF(rotation_output);
        Py_DECREF(translation_output);
        Py_DECREF(world_output);
        return nullptr;
    }
    PyTuple_SET_ITEM(result, 0, reinterpret_cast<PyObject*>(rotation_output));
    PyTuple_SET_ITEM(result, 1, reinterpret_cast<PyObject*>(translation_output));
    PyTuple_SET_ITEM(result, 2, reinterpret_cast<PyObject*>(world_output));
    return result;
}

PyObject* batch_beam_state_metrics(
    PyObject*,
    PyObject* args,
    PyObject* kwargs
) {
    PyObject* polygons_object = nullptr;
    PyObject* counts_object = nullptr;
    PyObject* rotations_object = nullptr;
    PyObject* translations_object = nullptr;
    PyObject* placed_object = nullptr;
    PyObject* match_errors_object = nullptr;
    PyObject* areas_object = nullptr;
    double angle_step = 1.0;
    double translation_step = 0.75;
    int workers = 1;
    static const char* keywords[] = {
        "polygons",
        "counts",
        "rotations",
        "translations",
        "placed",
        "match_errors",
        "areas",
        "angle_step_deg",
        "translation_step_mm",
        "workers",
        nullptr,
    };
    if (!PyArg_ParseTupleAndKeywords(
            args,
            kwargs,
            "OOOOOOO|ddi",
            const_cast<char**>(keywords),
            &polygons_object,
            &counts_object,
            &rotations_object,
            &translations_object,
            &placed_object,
            &match_errors_object,
            &areas_object,
            &angle_step,
            &translation_step,
            &workers)) {
        return nullptr;
    }
    if (angle_step <= 0.0 || translation_step <= 0.0 || workers < 1) {
        PyErr_SetString(
            PyExc_ValueError,
            "quantization steps and workers must be positive"
        );
        return nullptr;
    }

    PyArrayObject* polygons = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            polygons_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* counts = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            counts_object,
            NPY_INT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* rotations = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            rotations_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* translations = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            translations_object,
            NPY_FLOAT32,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* placed = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            placed_object,
            NPY_BOOL,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* match_errors = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            match_errors_object,
            NPY_DOUBLE,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    PyArrayObject* areas = reinterpret_cast<PyArrayObject*>(
        PyArray_FROM_OTF(
            areas_object,
            NPY_DOUBLE,
            NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST
        )
    );
    if (
        polygons == nullptr
        || counts == nullptr
        || rotations == nullptr
        || translations == nullptr
        || placed == nullptr
        || match_errors == nullptr
        || areas == nullptr
    ) {
        Py_XDECREF(polygons);
        Py_XDECREF(counts);
        Py_XDECREF(rotations);
        Py_XDECREF(translations);
        Py_XDECREF(placed);
        Py_XDECREF(match_errors);
        Py_XDECREF(areas);
        return nullptr;
    }

    const npy_intp piece_count = PyArray_DIM(polygons, 0);
    const npy_intp maximum_vertices = PyArray_DIM(polygons, 1);
    const npy_intp batch = PyArray_DIM(rotations, 0);
    const bool shapes_valid =
        PyArray_NDIM(polygons) == 3
        && PyArray_DIM(polygons, 2) == 2
        && piece_count >= 2
        && piece_count <= 4
        && PyArray_NDIM(counts) == 1
        && PyArray_DIM(counts, 0) == piece_count
        && PyArray_NDIM(rotations) == 4
        && PyArray_DIM(rotations, 1) == piece_count
        && PyArray_DIM(rotations, 2) == 2
        && PyArray_DIM(rotations, 3) == 2
        && PyArray_NDIM(translations) == 3
        && PyArray_DIM(translations, 0) == batch
        && PyArray_DIM(translations, 1) == piece_count
        && PyArray_DIM(translations, 2) == 2
        && PyArray_NDIM(placed) == 2
        && PyArray_DIM(placed, 0) == batch
        && PyArray_DIM(placed, 1) == piece_count
        && PyArray_NDIM(match_errors) == 1
        && PyArray_DIM(match_errors, 0) == batch
        && PyArray_NDIM(areas) == 1
        && PyArray_DIM(areas, 0) == piece_count;
    if (!shapes_valid) {
        Py_DECREF(polygons);
        Py_DECREF(counts);
        Py_DECREF(rotations);
        Py_DECREF(translations);
        Py_DECREF(placed);
        Py_DECREF(match_errors);
        Py_DECREF(areas);
        PyErr_SetString(PyExc_ValueError, "invalid beam metric array shapes");
        return nullptr;
    }
    const int* count_data = static_cast<const int*>(PyArray_DATA(counts));
    for (npy_intp piece = 0; piece < piece_count; ++piece) {
        if (count_data[piece] < 3 || count_data[piece] > maximum_vertices) {
            Py_DECREF(polygons);
            Py_DECREF(counts);
            Py_DECREF(rotations);
            Py_DECREF(translations);
            Py_DECREF(placed);
            Py_DECREF(match_errors);
            Py_DECREF(areas);
            PyErr_SetString(PyExc_ValueError, "invalid beam polygon count");
            return nullptr;
        }
    }

    npy_intp score_shape[] = {batch};
    npy_intp signature_shape[] = {batch, piece_count, 3};
    PyArrayObject* partial_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(1, score_shape, NPY_DOUBLE)
    );
    PyArrayObject* cheap_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(1, score_shape, NPY_DOUBLE)
    );
    PyArrayObject* feasible_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(1, score_shape, NPY_BOOL)
    );
    PyArrayObject* signature_output = reinterpret_cast<PyArrayObject*>(
        PyArray_SimpleNew(3, signature_shape, NPY_INT64)
    );
    if (
        partial_output == nullptr
        || cheap_output == nullptr
        || feasible_output == nullptr
        || signature_output == nullptr
    ) {
        Py_XDECREF(partial_output);
        Py_XDECREF(cheap_output);
        Py_XDECREF(feasible_output);
        Py_XDECREF(signature_output);
        Py_DECREF(polygons);
        Py_DECREF(counts);
        Py_DECREF(rotations);
        Py_DECREF(translations);
        Py_DECREF(placed);
        Py_DECREF(match_errors);
        Py_DECREF(areas);
        return nullptr;
    }

    const float* polygon_data = static_cast<const float*>(
        PyArray_DATA(polygons)
    );
    const float* rotation_data = static_cast<const float*>(
        PyArray_DATA(rotations)
    );
    const float* translation_data = static_cast<const float*>(
        PyArray_DATA(translations)
    );
    const npy_bool* placed_data = static_cast<const npy_bool*>(
        PyArray_DATA(placed)
    );
    const double* match_error_data = static_cast<const double*>(
        PyArray_DATA(match_errors)
    );
    const double* area_data = static_cast<const double*>(PyArray_DATA(areas));
    double* partial_data = static_cast<double*>(PyArray_DATA(partial_output));
    double* cheap_data = static_cast<double*>(PyArray_DATA(cheap_output));
    npy_bool* feasible_data = static_cast<npy_bool*>(
        PyArray_DATA(feasible_output)
    );
    npy_int64* signature_data = static_cast<npy_int64*>(
        PyArray_DATA(signature_output)
    );

    const npy_intp polygon_piece_stride = maximum_vertices * 2;
    const npy_intp rotation_state_stride = piece_count * 4;
    const npy_intp translation_state_stride = piece_count * 2;
    const npy_intp signature_state_stride = piece_count * 3;
    Py_BEGIN_ALLOW_THREADS
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(workers)
#endif
    for (npy_intp state = 0; state < batch; ++state) {
        std::vector<Point2d> points;
        points.reserve(static_cast<std::size_t>(piece_count * maximum_vertices));
        double pieces_area = 0.0;
        for (npy_intp piece = 0; piece < piece_count; ++piece) {
            npy_int64* signature = signature_data
                + state * signature_state_stride + piece * 3;
            if (!placed_data[state * piece_count + piece]) {
                signature[0] = 0;
                signature[1] = 0;
                signature[2] = 0;
                continue;
            }
            const float* rotation = rotation_data
                + state * rotation_state_stride + piece * 4;
            const float* translation = translation_data
                + state * translation_state_stride + piece * 2;
            signature[0] = static_cast<npy_int64>(std::nearbyint(
                std::atan2(
                    static_cast<double>(rotation[2]),
                    static_cast<double>(rotation[0])
                ) * 180.0 / 3.14159265358979323846 / angle_step
            ));
            signature[1] = static_cast<npy_int64>(std::nearbyint(
                static_cast<double>(translation[0]) / translation_step
            ));
            signature[2] = static_cast<npy_int64>(std::nearbyint(
                static_cast<double>(translation[1]) / translation_step
            ));
            pieces_area += area_data[piece];
            const float* polygon = polygon_data + piece * polygon_piece_stride;
            for (int vertex = 0; vertex < count_data[piece]; ++vertex) {
                const double x = polygon[vertex * 2];
                const double y = polygon[vertex * 2 + 1];
                points.push_back({
                    rotation[0] * x + rotation[1] * y + translation[0],
                    rotation[2] * x + rotation[3] * y + translation[1],
                });
            }
        }
        const std::array<double, 2> sides = minimum_bounding_sides(points);
        const double short_side = sides[0];
        const double long_side = sides[1];
        const double rectangle_area = std::max(1e-6, short_side * long_side);
        const double compactness = std::max(
            0.0,
            rectangle_area / std::max(1e-6, pieces_area) - 1.0
        );
        partial_data[state] = match_error_data[state] + 0.08 * compactness;
        const double fill_error = std::abs(
            1.0 - pieces_area / rectangle_area
        );
        double dimension_error = 0.0;
        if (long_side < 90.0) {
            dimension_error += (90.0 - long_side) / 90.0;
        } else if (long_side > 120.0) {
            dimension_error += (long_side - 120.0) / 120.0;
        }
        if (short_side < 50.0) {
            dimension_error += (50.0 - short_side) / 50.0;
        } else if (short_side > 90.0) {
            dimension_error += (short_side - 90.0) / 90.0;
        }
        cheap_data[state] = match_error_data[state]
            + fill_error * 3.5 + dimension_error * 8.0;
        feasible_data[state] = static_cast<npy_bool>(
            long_side <= 130.0 && short_side <= 105.0
        );
    }
    Py_END_ALLOW_THREADS

    Py_DECREF(polygons);
    Py_DECREF(counts);
    Py_DECREF(rotations);
    Py_DECREF(translations);
    Py_DECREF(placed);
    Py_DECREF(match_errors);
    Py_DECREF(areas);
    PyObject* result = PyTuple_New(4);
    if (result == nullptr) {
        Py_DECREF(partial_output);
        Py_DECREF(cheap_output);
        Py_DECREF(feasible_output);
        Py_DECREF(signature_output);
        return nullptr;
    }
    PyTuple_SET_ITEM(result, 0, reinterpret_cast<PyObject*>(partial_output));
    PyTuple_SET_ITEM(result, 1, reinterpret_cast<PyObject*>(cheap_output));
    PyTuple_SET_ITEM(result, 2, reinterpret_cast<PyObject*>(feasible_output));
    PyTuple_SET_ITEM(result, 3, reinterpret_cast<PyObject*>(signature_output));
    return result;
}

PyMethodDef methods[] = {
    {
        "best_shifted_overlap",
        reinterpret_cast<PyCFunction>(best_shifted_overlap),
        METH_VARARGS | METH_KEYWORDS,
        "Compute exact zero-padded Dice overlap over small translations."
    },
    {
        "batch_best_shifted_overlap",
        reinterpret_cast<PyCFunction>(batch_best_shifted_overlap),
        METH_VARARGS | METH_KEYWORDS,
        "Compute a batch of overlaps, using OpenMP across candidates."
    },
    {
        "outer_corner_metrics",
        reinterpret_cast<PyCFunction>(outer_corner_metrics),
        METH_VARARGS | METH_KEYWORDS,
        "Measure real outline angles at four rectangle corners."
    },
    {
        "batch_convex_overlap_areas",
        reinterpret_cast<PyCFunction>(batch_convex_overlap_areas),
        METH_VARARGS | METH_KEYWORDS,
        "Compute convex overlap areas in parallel across polygon pairs."
    },
    {
        "batch_edge_alignment_world",
        reinterpret_cast<PyCFunction>(batch_edge_alignment_world),
        METH_VARARGS | METH_KEYWORDS,
        "Align edge jobs and transform moving polygons in parallel."
    },
    {
        "batch_beam_state_metrics",
        reinterpret_cast<PyCFunction>(batch_beam_state_metrics),
        METH_VARARGS | METH_KEYWORDS,
        "Score, validate and quantize a batch of Beam search states."
    },
    {nullptr, nullptr, 0, nullptr}
};

PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "vision_fast",
    "Optional native acceleration for the vision puzzle solver.",
    -1,
    methods,
};

}  // namespace

PyMODINIT_FUNC PyInit_vision_fast() {
    import_array();
    return PyModule_Create(&module);
}
