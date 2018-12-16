// Given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval.  Optionally, one can request a derivative
// of the spline (0=no derivative, 1=first derivative, 2=2nd derivative).
template <class T>
inline T Spline<T>::cubicSplineUnitInterval(
    const T& position0, const T& position1, const T& tangent0,
    const T& tangent1, double normalizedTime, int derivative) {
  // TODO (Animation) Task 1a
	if (normalizedTime < 0 || normalizedTime > 1)
	{
		std::cout << "error" << std::endl;
	}

	double t = normalizedTime;
	double t_square = t * t,
		t_cube = t * t_square;
	double h00, h10, h01, h11;
	if (derivative == 0)
	{
		h00 = 2 * t_cube - 3 * t_square + 1;
		h10 = t_cube - 2 * t_square + t;
		h01 = -2 * t_cube + 3 * t_square;
		h11 = t_cube - t_square;
	}
	else if(derivative == 1)
	{
		h00 = 6 * t_square - 6 * t;
		h10 = 3 * t_square - 4 * t + 1;
		h01 = -6 * t_square + 6 * t;
		h11 = 3 * t_square - 2 * t;
	}
	else if(derivative == 2)
	{
		h00 = 12 * t - 6;
		h10 = 6 * t - 4;
		h01 = -12 * t + 6;
		h11 = 6 * t - 2;
	}
	if (derivative == 0 || derivative == 1 || derivative == 2)
	{
		return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
	}
	
  return T();
}

// Returns a state interpolated between the values directly before and after the
// given time.
template <class T>
inline T Spline<T>::evaluate(double time, int derivative) {
  // TODO (Animation) Task 1b
  if (knots.size() < 1)
    return T();
  else if (knots.size() == 1)
  {
	  if (derivative == 0)
	  {
		  return knots.begin()->second;
	  }
	  else
	  {
		  return T();
	  }
  }
  // At least 2 knots
  else
  {
	  if (time <= knots.begin()->first)
	  {
		  if (derivative == 0)
		  {
			  return knots.begin()->second;
		  }
		  else
		  {
			  return T();
		  }
	  }
	 
	  if (time >= knots.rbegin()->first)
	  {
		  if (derivative == 0)
		  {
			  return knots.rbegin()->second;
		  }
		  else
		  {
			  return T();
		  }
	  }

	  // General case
	  auto k2 = knots.upper_bound(time);
	  auto k1 = k2;
	  k1--;
	  auto k0 = k1;
	  k0--;
	  auto k3 = k2;
	  k3++;

	  double total_interval = k2->first - k1->first;
	  double normalizedTime = (time - k1->first) / total_interval;
	  T tangent0, tangent1;
	  if (k1 == knots.begin()) {
		  tangent0 = (k2->second - k1->second) / (k2->first - k1->first) * total_interval;
	  }
	  else {
		  tangent0 = (k2->second - k0->second) / (k2->first - k0->first) * total_interval;
	  }
	  if (k3 == knots.end()) {
		  tangent1 = (k2->second - k1->second) / (k2->first - k1->first) * total_interval;
	  }
	  else {
		  tangent1 = (k3->second - k1->second) / (k3->first - k1->first) * total_interval;
	  }

	  auto result = cubicSplineUnitInterval(k1->second, k2->second, tangent0, tangent1, normalizedTime, derivative);
	  if (derivative == 1)
	  {
		  result = result / total_interval;
	  }
	  else if(derivative == 2)
	  {
		  result = result / (total_interval * total_interval);
	  }
	  return result;
  }
  return T();
}

// Removes the knot closest to the given time,
//    within the given tolerance..
// returns true iff a knot was removed.
template <class T>
inline bool Spline<T>::removeKnot(double time, double tolerance) {
  // Empty maps have no knots.
  if (knots.size() < 1) {
    return false;
  }

  // Look up the first element > or = to time.
  typename std::map<double, T>::iterator t2_iter = knots.lower_bound(time);
  typename std::map<double, T>::iterator t1_iter;
  t1_iter = t2_iter;
  t1_iter--;

  if (t2_iter == knots.end()) {
    t2_iter = t1_iter;
  }

  // Handle tolerance bounds,
  // because we are working with floating point numbers.
  double t1 = (*t1_iter).first;
  double t2 = (*t2_iter).first;

  double d1 = fabs(t1 - time);
  double d2 = fabs(t2 - time);

  if (d1 < tolerance && d1 < d2) {
    knots.erase(t1_iter);
    return true;
  }

  if (d2 < tolerance && d2 < d1) {
    knots.erase(t2_iter);
    return t2;
  }

  return false;
}

// Sets the value of the spline at a given time (i.e., knot),
// creating a new knot at this time if necessary.
template <class T>
inline void Spline<T>::setValue(double time, T value) {
  knots[time] = value;
}

template <class T>
inline T Spline<T>::operator()(double time) {
  return evaluate(time);
}
