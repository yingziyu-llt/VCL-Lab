#include "Labs/5-Visualization/tasks.h"

#include <numbers>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    struct CoordinateStates {
        // your code here
        int                                  current_var;
        std::vector<float>                   min_values;
        std::vector<float>                   max_values;
        std::vector<std::pair<float, float>> ranges;
        CoordinateStates() {
            current_var = -1;
            min_values  = std::vector<float>(7, 100000);
            max_values  = std::vector<float>(7, 0);
            ranges      = std::vector<std::pair<float, float>>(7, { 0, 1 });
        }
    };

    float getCarValue(Car const & c, int const & var) {
        switch (var) {
        case 0:
            return c.cylinders;
        case 1:
            return c.displacement;
        case 2:
            return c.weight;
        case 3:
            return c.horsepower;
        case 4:
            return c.acceleration;
        case 5:
            return c.mileage;
        case 6:
            return c.year;
        default:
            return 0;
        }
    }
    std::string get_name(int const & var) {
        switch (var) {
        case 0:
            return "cylinders";
        case 1:
            return "displacement";
        case 2:
            return "weight";
        case 3:
            return "horsepower";
        case 4:
            return "acceleration";
        case 5:
            return "mileage";
        case 6:
            return "year";
        default:
            return "";
        }
    }
    glm::vec4 linear_interpolation(glm::vec4 const & a, glm::vec4 const & b, float const & t) {
        if (t < 0) return a;
        if (t > 1) return b;
        return a * (1 - t) + b * t;
    }

    bool PaintParallelCoordinates(Common::ImageRGB & input, InteractProxy const & proxy, std::vector<Car> const & data, bool force) {
        // your code here
        // for example:
        //   static CoordinateStates states(data);
        //   SetBackGround(input, glm::vec4(1));
        //   ...
        const int   total_vars    = 7;
        const float left_margin   = 0.1f;
        const float right_margin  = 0.90f;
        const float top_margin    = 0.1f;
        const float bottom_margin = 0.90f;

        static CoordinateStates states;
        if (states.current_var == -1) {
            for (auto const & car : data) {
                for (int i = 0; i < total_vars; i++) {
                    float value          = getCarValue(car, i);
                    states.min_values[i] = std::min(states.min_values[i], value);
                    states.max_values[i] = std::max(states.max_values[i], value);
                }
            }
            for (int i = 0; i < total_vars; i++) {
                states.ranges[i] = { states.min_values[i], states.max_values[i] };
            }
            states.current_var = 0;
        }

        if(proxy.IsHovering()) {
            if(proxy.IsClicking()){
                //printf("clicking\n");
                float pos = (proxy.MousePos().x - left_margin) / (right_margin - left_margin);
                //printf("pos: %f", pos * 6);
                //printf("diff: %f\n",fabs(pos * 6 - (int)(pos * 6)));
                if(fabs(pos * 6 - (int)(pos * 6 + 0.5)) < 0.2) {
                    states.current_var = (int)(pos * 6 + 0.3);
                    states.ranges[states.current_var].first = states.min_values[states.current_var];
                    states.ranges[states.current_var].second = states.max_values[states.current_var];
                }
            }
            if(proxy.IsDragging()){
                float pos = (proxy.MousePos().x - left_margin) / (right_margin - left_margin);
                //printf("dragging\n");
                //printf("range: %f %f\n", proxy.DraggingStartPoint().y, proxy.MousePos().y);
                if(fabs(pos * 6 - (int)(pos * 6 + 0.5)) < 0.4) {
                states.current_var = (int)(pos * 6 + 0.5);
                float min_y = std::min(proxy.DraggingStartPoint().y, proxy.MousePos().y);
                float max_y = std::max(proxy.DraggingStartPoint().y, proxy.MousePos().y);
                
                max_y = (max_y - top_margin) / (bottom_margin - top_margin);
                min_y = (min_y - top_margin) / (bottom_margin - top_margin);
                states.ranges[states.current_var].first = std::max(states.min_values[states.current_var], min_y * (states.max_values[states.current_var] - states.min_values[states.current_var]) + states.min_values[states.current_var]);
                states.ranges[states.current_var].second = std::min(states.max_values[states.current_var], max_y * (states.max_values[states.current_var] - states.min_values[states.current_var]) + states.min_values[states.current_var]);
                }
            }
        }
        static const glm::vec4 gray = glm::vec4(111,111,111,255) / 255.0f;
        static const glm::vec4 red = glm::vec4(255,70,0,255) / 255.0f;
        static const glm::vec4 blue = glm::vec4(0,70,255,255) / 255.0f;
        SetBackGround(input, glm::vec4(1));
        
        for (const auto& car : data) {
            bool is_in_range = true;
            for (int i = 0; i < total_vars; i++) {
                float value = getCarValue(car, i);
                if (value < states.ranges[i].first || value > states.ranges[i].second) {
                    is_in_range = false;
                    break;
                }
            }
            for (int i = 0;i < total_vars - 1;i++) {
                float x = left_margin + (right_margin - left_margin) * i / (float)(total_vars - 1);
                float y = top_margin + (bottom_margin - top_margin) * (getCarValue(car, i) - states.min_values[i]) / (states.max_values[i] - states.min_values[i]);
                float nxt_y = top_margin + (bottom_margin - top_margin) * (getCarValue(car, i + 1) - states.min_values[i + 1]) / (states.max_values[i + 1] - states.min_values[i + 1]);
                glm::vec4 color = linear_interpolation(red, blue, (getCarValue(car,states.current_var) - states.min_values[states.current_var]) / (states.max_values[states.current_var] - states.min_values[states.current_var]));
                if (!is_in_range)
                    color = gray,color.w = 0.5;
                DrawLine(input, color, glm::vec2(x, y), glm::vec2(left_margin + (right_margin - left_margin) * (i + 1) / (float)(total_vars - 1), nxt_y), 0.005);
            }
        }
        for (int i = 0; i < total_vars; i++) {
            float x = left_margin + (right_margin - left_margin) * i / (float)(total_vars - 1);
            float t = (states.ranges[i].first + states.ranges[i].second - 2 * states.min_values[i]) / 2.0f / (states.max_values[i] - states.min_values[i]);
            glm::vec4 col = i == states.current_var ? linear_interpolation(red,blue,t) : gray;
            col.w = 0.5;
            DrawFilledRect(input,col, glm::vec2(x - 0.015, top_margin + (states.ranges[i].first - states.min_values[i]) / (states.max_values[i] - states.min_values[i]) * (bottom_margin - top_margin)), glm::vec2(0.03, (states.ranges[i].second - states.ranges[i].first) / (states.max_values[i] - states.min_values[i]) * (bottom_margin - top_margin)));
            PrintText(input,glm::vec4(0,0,0,1),glm::vec2(x + 0.01, top_margin - 0.04),0.02,get_name(i));
            DrawLine(input,glm::vec4(0,0,0,1),glm::vec2(x,top_margin),glm::vec2(x,bottom_margin),0.005);
            PrintText(input,glm::vec4(0,0,0,1),glm::vec2(x - 0.01, top_margin + (states.ranges[i].first - states.min_values[i]) / (states.max_values[i] - states.min_values[i]) * (bottom_margin - top_margin) - 0.02),0.02,std::to_string(states.ranges[i].first));
            PrintText(input,glm::vec4(0,0,0,1),glm::vec2(x - 0.01, top_margin + (states.ranges[i].second - states.min_values[i]) / (states.max_values[i] - states.min_values[i]) * (bottom_margin - top_margin) + 0.02),0.02,std::to_string(states.ranges[i].second));
        }
        return true;
    }

    float kernel(float x, float y, float sigma = 20.0,float r = 4.0) {
        float s = sqrt(x * x + y * y);
        if (fabs(s) < r) return 1.0f;
        float t = fabs(s) - r;
        return exp(-t * t / (2 * sigma * sigma));
    }

    void LIC(ImageRGB & output, Common::ImageRGB const & noise, VectorField2D const & field, int const & step) {
        // your code here
        int n = output.GetSizeX();
        int m = output.GetSizeY();
        float v_max = 0;
        for (int x = 0;x < n;x++) {
            for (int y = 0;y < m;y++) {
                v_max = std::max(v_max,glm::length(field.At(x,y)));
            }
        }
        for (int x0 = 0;x0 < n;x0++) {
            for (int y0 = 0;y0 < m;y0++) {
                float x = x0,y = y0;
                glm::vec2 dir = field.At(x,y);
                glm::vec3 val = glm::vec4(0);
                float dx = 0,dy = 0;
                float total_weight = 0;
                for (int i = 0; i < step;i++) {
                    dx = field.At(x,y).x;
                    dy = field.At(x,y).y;
                    float dt = 1;
                    if (dx > 0) 
                        dt = (std::floor(x) + 1.0f - x) / dx;
                    else 
                        dt = (x - (std::ceil(x) - 1)) / (-dx);
                    if (dy > 0)
                        dt = std::min(dt, (std::floor(y) + 1.0f - y) / dy);
                    else 
                        dt = std::min(dt, (y - (std ::ceil(y) - 1)) / (-dy));
                    x = std::max(0.0f,std::min(n - 1.0f,x + dx * dt));
                    y = std::max(0.0f,std::min(m - 1.0f,y + dy * dt));
                    float weight = kernel(x - x0,y - y0);
                    val += weight * noise.At(x,y);
                    total_weight += weight;
                }
                
                x = x0,y = y0;
                for (int i = 0;i < step;i++){
                    dx = field.At(x,y).x;
                    dy = field.At(x,y).y;
                    float dt = 1;
                    if (dx > 0) 
                        dt = (std::floor(x) + 1.0f - x) / dx;
                    else 
                        dt = (x - (std::ceil(x) - 1)) / (-dx);
                    if (dy > 0)
                        dt = std::min(dt, (std::floor(y) + 1.0f - y) / dy);
                    else 
                        dt = std::min(dt, (y - (std ::ceil(y) - 1)) / (-dy));
                    x = std::max(0.0f,std::min(n - 1.0f,x - dx * dt));
                    y = std::max(0.0f,std::min(m - 1.0f,y - dy * dt));
                    float weight = kernel(x - x0,y - y0);
                    val += weight * noise.At(x,y);
                    total_weight += weight;
                }
                val += kernel(0,0) * noise.At(x0,y0);
                total_weight += kernel(0,0);
                float noi = val.r / total_weight;
                glm::vec4 color = linear_interpolation(glm::vec4(0,0,1,1),glm::vec4(1,0,0,1),glm::length(field.At(x0,y0)) / v_max);
                color = color * noi;
                color += glm::vec4(1) * 0.1f;
                output.At(x0,y0) = color;
                
            }
        }
    }
}; // namespace VCX::Labs::Visualization