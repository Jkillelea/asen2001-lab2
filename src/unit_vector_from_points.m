function u_vector = unit_vector_from_points(vector_a, to_or_from, vector_b)
  if to_or_from == 'to'
    u_vector = to_unit_vector(vector_b - vector_a);
  elseif to_or_from == 'from'
    u_vector = to_unit_vector(vector_a - vector_b);
  else
    error('Must specify direction as either "to" or "from"!');
  end
end
