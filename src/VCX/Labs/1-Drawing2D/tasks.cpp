#include <random>

#include <algorithm>

#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;

namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::random_device seed;
        std::mt19937 gen(seed());
        std::uniform_int_distribution<> my_rand(-500,500);
        glm::vec3 noise(.0,.0,.0);

        output = input;
        for(int i = 0;i < input.GetSizeX();i++)
            for(int j = 0;j < input.GetSizeY();j++) {
                 double rand = my_rand(gen) / 1000.0;
                noise = glm::vec3(rand,rand,rand);
                output.At(i,j) = input.At(i,j) + noise;
            }
        DitheringThreshold(output,output);
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        output = input;
        
        for(int i = 0;i < input.GetSizeX();i++)
            for(int j = 0;j < input.GetSizeY();j++) {
                output.At(i,j) = input.At(i,j) + noise.At(i,j) - glm::vec3(0.5,0.5,0.5);
            }
        DitheringThreshold(output,output);
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        for (int i = 0;i < input.GetSizeX();i++) {
            for (int j = 0;j < input.GetSizeY();j++) {
                int curr_col = (int)(input.At(i,j).r * 10);
                int ii = i * 3 + 1,jj = j * 3 + 1;
                for (int k = 1;k <= curr_col;k++) {
                    if (k == 1) output.At(ii,jj) = glm::vec3(1,1,1);
                    if (k == 2) output.At(ii - 1,jj) = glm::vec3(1,1,1);
                    if (k == 3) output.At(ii,jj + 1) = glm::vec3(1,1,1);
                    if (k == 4) output.At(ii + 1,jj) = glm::vec3(1,1,1);
                    if (k == 5) output.At(ii + 1,jj - 1) = glm::vec3(1,1,1);
                    if (k == 6) output.At(ii - 1,jj + 1) = glm::vec3(1,1,1);
                    if (k == 7) output.At(ii - 1,jj - 1) = glm::vec3(1,1,1);
                    if (k == 8) output.At(ii + 1,jj + 1) = glm::vec3(1,1,1);
                    if (k == 9) output.At(ii,jj - 1) = glm::vec3(1,1,1);
                }
            } 
        }

    }
    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        glm::vec3 tmp(0,0,0);
        output = input;
        for (int i = 0;i < input.GetSizeX();i++) {
            for (int j = 0;j < input.GetSizeY();j++) {
                glm::vec3 error(0,0,0);
                glm::vec3 old_pix = output.At(i,j);
                if(old_pix.r > 0.5) {
                    error = old_pix -  glm::vec3(1,1,1);
                    output.At(i,j) = glm::vec3(1,1,1);
                } else {
                    error = old_pix - glm::vec3(0,0,0);
                    output.At(i,j) = glm::vec3(0,0,0);
                }
                
                auto fn = [&](int ii,int jj,float rate) -> glm::vec3 {
                    tmp = output.At(ii,jj);
                    return tmp + rate * error;
                };
                
                if(j < input.GetSizeY() - 1)
                    output.At(i,j + 1) = fn(i,j + 1,7.0 / 16.0);
                if(j > 0 && i < input.GetSizeX() - 1)
                    output.At(i + 1,j - 1) = fn(i + 1,j - 1,3.0 / 16.0);
                if(i < input.GetSizeX() - 1)
                    output.At(i + 1,j) = fn(i + 1,j,5.0 / 16.0);
                if(i < input.GetSizeX() - 1 && j < input.GetSizeY() - 1)
                    output.At(i + 1,j + 1) = fn(i + 1,j + 1,1.0 / 16.0);
            }
        }
    }

    /******************* 2.Image Filtering *****************/

    void Convolution(
        ImageRGB &output,
        ImageRGB const &input,
        std::array<std::array<glm::vec3, 3>, 3> const & kernel) {
            glm::vec3 sum(0,0,0);
            for(int i = 0;i < 3;i++)
                for(int j = 0;j < 3;j++) {
                    sum += kernel[i][j];
                }
            for (int i = 0;i < input.GetSizeX() - 2;i++) {
                for (int j = 0;j < input.GetSizeY() - 2;j++) {
                    glm::vec3 tmp(0,0,0);
                    for (int ii = 0;ii < 3;ii++) {
                        for (int jj = 0;jj < 3;jj++) {
                            glm::vec3 pix = input.At(i + ii,j + jj);
                            tmp += kernel[ii][jj] * pix;
                        }
                    }
                    if(sum.r != 0 && sum.g != 0 && sum.b != 0)
                        tmp /= sum;
                    output.At(i,j) = glm::abs(tmp);
                }
            }
    }

    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        std::array<std::array<glm::vec3, 3>, 3> kernel = {
            {
                {glm::vec3(1,1,1),glm::vec3(2,2,2),glm::vec3(1,1,1)},
                {glm::vec3(2,2,2),glm::vec3(4,4,4),glm::vec3(2,2,2)},
                {glm::vec3(1,1,1),glm::vec3(2,2,2),glm::vec3(1,1,1)}
            }
        };
        Convolution(output, input, kernel);
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::array<std::array<glm::vec3, 3>, 3> horizontal_kernal = {
            {
                {glm::vec3(-1,-1,-1),glm::vec3(0,0,0),glm::vec3(1,1,1)},
                {glm::vec3(-2,-2,-2),glm::vec3(0,0,0),glm::vec3(2,2,2)},
                {glm::vec3(-1,-1,-1),glm::vec3(0,0,0),glm::vec3(1,1,1)}
            }
        };
        std::array<std::array<glm::vec3, 3>, 3> vertical_kernal = {
            {
                {glm::vec3(1,1,1),glm::vec3(2,2,2),glm::vec3(1,1,1)},
                {glm::vec3(0,0,0),glm::vec3(0,0,0),glm::vec3(0,0,0)},
                {glm::vec3(-1,-1,-1),glm::vec3(-2,-2,-2),glm::vec3(-1,-1,-1)}
            }
        };
        ImageRGB horizontal_edge = input,vertical_edge = input;
        Convolution(horizontal_edge, input, vertical_kernal);
        Convolution(vertical_edge, input, horizontal_kernal);
        for (int i = 0;i < input.GetSizeX();i++) {
            for (int j = 0;j < input.GetSizeY();j++) {
                glm::vec3 tmp1,tmp2,res;
                tmp1 = horizontal_edge.At(i,j);
                tmp2 = vertical_edge.At(i,j);
                tmp1 = tmp1 * tmp1;
                tmp2 = tmp2 * tmp2;
                res = glm::sqrt(tmp1 + tmp2);
                output.At(i,j) = res;
            }
        }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        
        // vanilla algorithm

        /*int x0 = p0.x,y0 = p0.y,x1 = p1.x,y1 = p1.y;

        for(int i = x0;i <= x1;i++)
        {
            float ny = (y1 - y0) / (float)(x1 - x0) * (i - x0) + y0;
            canvas.At(i,(int)round(ny)) = color;
        }*/

        // Bresenham
        int x0 = p0.x, y0 = p0.y, x1 = p1.x, y1 = p1.y;
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1; 
        int err = dx - dy;
        while (true) {
            canvas.At(x0, y0) = color;

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        glm::ivec2 p[3] = {p0, p1, p2};
        std::sort(p, p + 3, [](glm::ivec2 const & a, glm::ivec2 const & b) { return a.y < b.y; });
        for(int y = p[0].y;y < p[1].y;++y) {
            int x1 = p[0].x + (y - p[0].y) * (p[1].x - p[0].x) / (p[1].y - p[0].y);
            int x2 = p[0].x + (y - p[0].y) * (p[2].x - p[0].x) / (p[2].y - p[0].y);
            if(x1 > x2) std::swap(x1, x2);
            for(int x = x1;x <= x2;++x) {
                canvas.At(x, y) = color;
            }
        }
        for(int y = p[1].y;y <= p[1].y;++y) {
            int x1,x2;
            if(p[1].y == p[2].y) {
                x1 = p[1].x;
                x2 = p[2].x;
            } else if(p[1].y == p[0].y) {
                x1 = p[0].x;
                x2 = p[1].x;
            } else {
                x1 = p[0].x + (y - p[0].y) * (p[1].x - p[0].x) / (p[1].y - p[0].y);
                x2 = p[0].x + (y - p[0].y) * (p[2].x - p[0].x) / (p[2].y - p[0].y);
            }
            if(x1 > x2) std::swap(x1, x2);
            for(int x = x1;x <= x2;++x) {
                canvas.At(x, y) = color; 
            }
        }
        for(int y = p[1].y + 1;y <= p[2].y;++y) {
            int x1 = p[1].x + (y - p[1].y) * (p[2].x - p[1].x) / (p[2].y - p[1].y);
            int x2 = p[0].x + (y - p[0].y) * (p[2].x - p[0].x) / (p[2].y - p[0].y);
            if(x1 > x2) std::swap(x1, x2);
            for(int x = x1;x <= x2;++x) {
                canvas.At(x, y) = color;
            }
        }
        
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        //SSAA algorithm

        ImageRGB temp = VCX::Labs::Common::CreatePureImageRGB(input.GetSizeX(),input.GetSizeY(),glm::vec3(0,0,0));
        for(int i = 0;i < input.GetSizeX();i += rate) {
            for(int j = 0;j < input.GetSizeY();j += rate) {
                int x = i + rate / 2,y = j + rate / 2;
                int size = rate * rate;
                glm::vec3 color(0,0,0);
                for(int k = 0;k < rate;k++) {
                    for(int l = 0;l < rate;l++) {
                        if(i + k < input.GetSizeX() && j + l < input.GetSizeY())
                            color += input.At(i + k,j + l);
                        else
                            size--;
                    }
                }
                color /= size;
                for(int k = 0;k < rate;k++) {
                    for(int l = 0;l < rate;l++) {
                        if(i + k < input.GetSizeX() && j + l < input.GetSizeY())
                            temp.At(x + k,y + l) = color;
                    }
                }
            }
        }
        output = VCX::Labs::Common::CreatePureImageRGB(320,320,{0.0f,0.0f,0.0f});
        for(int i = 0;i < output.GetSizeX();i++) {
            for(int j = 0;j < output.GetSizeY();j++) {
                float newx = i * 1.0 * input.GetSizeX() / output.GetSizeX(),newy = j * 1.0 * input.GetSizeY() / output.GetSizeY();
                auto interpolation = [&](float x,float y) -> glm::vec3 {
                    int xx = int(x),yy = int(y);
                    glm::vec3 v1 = temp.At(xx,yy),v2 = temp.At(xx + 1,yy),v3 = temp.At(xx,yy + 1),v4 = temp.At(xx + 1,yy + 1);
                    return v1 * (1 - x) * (1 - y) + v2 * x * (1 - y) + v3 * (1 - x) * y + v4 * x * y;
                };
                output.At(i,j) = interpolation(newx,newy);
            }
        }
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
            if(points.size() == 1) {
                return points[0];
            }
            std::vector<glm::vec2> newPoints(points.size() - 1);
            for(int i = 0;i < newPoints.size();++i) {
                newPoints[i] = (1 - t) * points[i] + t * points[i + 1];
            }
            return CalculateBezierPoint(newPoints, t);
    }
} // namespace VCX::Labs::Drawing2D