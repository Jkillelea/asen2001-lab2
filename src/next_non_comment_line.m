% Copyright Jacob Killelea, ASEN 2001 Lab 2, Fall 2016
function line = next_non_comment_line(fileID)
  % returns the next line that doesn't begin with a #
  % if this function is called repeatedly, it will return a new line each time, until EOF
  line = fgets(fileID);
  while (line(1) == '#')
    line = fgets(fileID);
  end
end
