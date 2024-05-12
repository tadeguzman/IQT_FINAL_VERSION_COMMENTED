syms lat1 lon1 d R brng v t real;

lat2 = asin(sin(lat1/1E7)*cos(v/1E3*t/R) + cos(lat1/1E7)*sin(v/1E3*t/R)*cos(brng/1E5))
lon2 = lon1/1E7 + atan2(sin(brng/1E5)*sin(v/1E3*t/R)*cos(lat1/1E7), cos(v/1E3*t/R)-sin(lat1/1E7)*sin(lat2))

f = simplify([lat2*1E7; lon2*1E7; v; brng])

j = simplify(jacobian(f, [lat1; lon1; v; brng]))
