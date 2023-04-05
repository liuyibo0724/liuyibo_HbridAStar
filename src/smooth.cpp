#include "smooth.h"

using namespace HybridAStar;

//方向Prim是否为前进区间
inline bool isAdv(int prim) { return prim < 3 && prim >= 0; }

//判断是否为交点（内联）
// bool isCrop(std::vector<Node3D> path, int i)
// {
//     if(path[i].getPrim == -1) return false;
//     else if(path[i].getPrim == -2) return true;
//     bool revp2Adv = isAdv(path[i - 2].getPrim()) ? true : false;
//     bool revp1Adv = isAdv(path[i - 1].getPrim()) ? true : false;
//     bool selfpAdv = isAdv(path[i].getPrim()) ? true : false;
//     bool prep1Adv = isAdv(path[i + 1].getPrim()) ? true : false;
//     bool prep2Adv = isAdv(path[i + 2].getPrim()) ? true : false;
//     if(revp2Adv != revp1Adv || revp1Adv != selfpAdv || selfpAdv != prep1Adv || prep1Adv != prep2Adv)
//     {
//         return true;
//     }
//     return false;
// }
inline bool isCusp(std::vector<Node3D> path, int i)
{
    if(path[i].getPrim() == -1)
    {
        return false;
    }
    else if(path[i].getPrim() == -2)
    {
        return true;
    }
    bool revim2 = path[i - 2].getPrim() > 3 ? true : false;
    bool revim1 = path[i - 1].getPrim() > 3 ? true : false;
    bool revi   = path[i].getPrim() > 3 ? true : false;
    bool revip1 = path[i + 1].getPrim() > 3 ? true : false;
    //  bool revip2 = path[i + 2].getPrim() > 3 ? true : false;
    if (revim2 != revim1 || revim1 != revi || revi != revip1)
    {
        return true;
    }
    return false;
}

//曲率项
Vector2D Smoother::curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2) 
{
  Vector2D gradient;
  // the vectors between the nodes
  const Vector2D& delta_x_im1 = x_im1 - x_im2;
  const Vector2D& delta_x_i = x_i - x_im1;
  const Vector2D& delta_x_ip1 = x_ip1 - x_i;
  const Vector2D& delta_x_ip2 = x_ip2 - x_ip1;

  // ensure that the absolute values are not null
  if (delta_x_im1.length() > 0 && delta_x_i.length() > 0 && delta_x_ip1.length() > 0 && delta_x_ip2.length() > 0) {
    // the angular change at the node
    auto compute_kappa = [](const Vector2D& delta_x_0, const Vector2D& delta_x_1, float& delta_phi, float& kappa) {
        delta_phi = std::acos(clamp(delta_x_0.dot(delta_x_1) / (delta_x_0.length() * delta_x_1.length()), -1, 1));
        kappa = delta_phi / delta_x_0.length();
    };
    float delta_phi_im1, kappa_im1;
    compute_kappa(delta_x_im1, delta_x_i, delta_phi_im1, kappa_im1);
    float delta_phi_i, kappa_i;
    compute_kappa(delta_x_i, delta_x_ip1, delta_phi_i, kappa_i);
    float delta_phi_ip1, kappa_ip1;
    compute_kappa(delta_x_ip1, delta_x_ip2, delta_phi_ip1, kappa_ip1);

    // if the curvature is smaller then the maximum do nothing
    if (kappa_i <= kappaMax) {
      Vector2D zeros;
      return zeros;
    } else {
      auto compute_d_delta_phi = [](const float delta_phi){
          return -1. / std::sqrt(1. - std::pow(std::cos(delta_phi), 2));
      };

      const float& d_delta_phi_im1 = compute_d_delta_phi(delta_phi_im1);
      const Vector2D& d_cos_delta_phi_im1 = delta_x_im1.ort(delta_x_i) / (delta_x_im1.length() * delta_x_i.length());
      const Vector2D& d_kappa_im1 = 1. / delta_x_im1.length() * d_delta_phi_im1 * d_cos_delta_phi_im1;
      const Vector2D& kim1 = 2. * (kappa_im1 - kappaMax) * d_kappa_im1;

      const float& d_delta_phi_i = compute_d_delta_phi(delta_phi_i);
      const Vector2D& d_cos_delta_phi_i = delta_x_ip1.ort(delta_x_i) / (delta_x_ip1.length() * delta_x_i.length()) -
                                          delta_x_i.ort(delta_x_ip1) / (delta_x_i.length() * delta_x_ip1.length());
      const Vector2D& d_kappa_i = 1. / delta_x_i.length() * d_delta_phi_i * d_cos_delta_phi_i -
                                  delta_phi_i / std::pow(delta_x_i.length(), 3) * delta_x_i;
      const Vector2D& ki = 2. * (kappa_i - kappaMax) * d_kappa_i;

      const float& d_delta_phi_ip1 = compute_d_delta_phi(delta_phi_ip1);
      const Vector2D& d_cos_delta_phi_ip1 = -delta_x_ip2.ort(delta_x_ip1) / (delta_x_ip2.length() * delta_x_ip1.length());
      const Vector2D& d_kappa_ip1 = 1. / delta_x_ip1.length() * d_delta_phi_ip1 * d_cos_delta_phi_ip1 +
                                    delta_phi_ip1 / std::pow(delta_x_ip1.length(), 3) * delta_x_ip1;
      const Vector2D& kip1 = 2. * (kappa_ip1 - kappaMax) * d_kappa_ip1;

      // calculate the gradient
      gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

      if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
        std::cout << "nan values in curvature term" << std::endl;
        Vector2D zeros;
        return zeros;
      }
      // return gradient of 0
      else {
        return gradient;
      }
    }
  }
  // return gradient of 0
  else {
    std::cout << "abs values not larger than 0" << std::endl;
    Vector2D zeros;
    return zeros;
  }
}

//光顺项
Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
    return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

//障碍物项
Vector2D Smoother::obstacleTerm(Vector2D xi)
{
    Vector2D gradient;
    // the distance to the closest obstacle from the current node
    float obsDst = voronoi.getDistance(xi.getX(), xi.getY());
    // the vector determining where the obstacle is
    int x = (int)xi.getX();
    int y = (int)xi.getY();
    // if the node is within the map
    if (x < width && x >= 0 && y < height && y >= 0)
    {
        //从当前点xi到最近障碍点的向量
        Vector2D obsVct(xi.getX() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstX,
                        xi.getY() - voronoi.data[(int)xi.getX()][(int)xi.getY()].obstY);

        // the closest obstacle is closer than desired correct the path for that
        if (obsDst < obsDMax) {
            return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
        }
    }
    return gradient;//有潜在风险，前面没有赋值
}

//找到轨迹点列
// void Smoother::tracePath(Node3D *node, int i, std::vector<Node3D> path)
// {
//     if(node == nullptr)
//     {
//         this->m_path = path;
//         return;
//     }
//     i ++;
//     path.push_back(*node);
//     tracePath(node->getPred(), i, path);
// }
void Smoother::tracePath(Node3D *node, int i, std::vector<Node3D> path)
{
    while(node != nullptr)
    {
        float x = node->getX();
        float y = node->getY();
        float t = node->getT();
        path.push_back(Node3D(x, y, t, 0, 0, nullptr, 0));
        auto tmp_ptr = node->pred;
        node = tmp_ptr;
    }
    this->m_path = path;
}

//核心轨迹平滑函数
void Smoother::smoothPath(DynamicVoronoi& voronoi)
{
    this->voronoi = voronoi;
    this->width = voronoi.getSizeX();
    this->height = voronoi.getSizeY();
    int iterations = 0;
    int maxIterations = 500;    //最大迭代次数
    int pathLength = m_path.size(); //路径总长度
    std::vector<Node3D> newPath = m_path;

    float totalWeight = wSmoothness + wCurvature + wVoronoi + wObstacle;   //四项权重

    while(iterations < maxIterations)
    {
        for(int i = 2; i < pathLength - 2; i ++)
        {
            float H = newPath[i].getH();
            //后面2个点，当前点，前面2个点
            Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
            Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
            Vector2D xi(newPath[i].getX(), newPath[i].getY());
            Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
            Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
            Vector2D correction;

            if (isCusp(newPath, i)) { continue; }//若为交点，不做平滑

            correction = correction - obstacleTerm(xi);
            if (!isOnGrid(xi + correction)) continue;   //假如校正方向超出当前监视的网格范围，不做处理

            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            if (!isOnGrid(xi + correction)) continue;

            correction = correction - curvatureTerm(xim2, xim1, xi, xip1, xip2);
            if (!isOnGrid(xi + correction)) continue; 

            auto update = alpha * correction/totalWeight;
            xi = xi + update;
            newPath[i].setX(xi.getX());
            newPath[i].setY(xi.getY());

            Vector2D Dxi = xi - xim1;
            newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
        }
        iterations ++;
    }
    m_path = newPath;
}