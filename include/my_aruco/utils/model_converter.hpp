


namespace my_aruco::utils
{

using InputValue = double;
using OutputValue = double;

struct  ModelParameters
{
  ModelParameters() = delete;
  ModelParameters(const double _r, const double _b) : r(_r), (_b) {}

  

  double r = 0.0;
  double b = 0.0;
  /* data */
};


void vr_vl_to_v_w(const InputValue &vl, const InputValue &vr, OutputValue &v, OutputValue &w) {

}
void v_w_to_vr_vl(const InputValue& v, const InputValue& w, OutputValue &v, OutputValue &w) {

}

  
} // namespace my_aruco
