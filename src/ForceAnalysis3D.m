function [barforces, reacforces] = ForceAnalysis3D(Joints_Array, MemberConnectivity_Array, ReactionJoints_Array, ReactionVector_Array, LoadJoints_Array, LoadVectors_Array)
% function [barforces, reacforces] = forceanalysis3D(Joints_Array, MemberConnectivity_Array, ReactionJoints_Array, ReactionVector_Array, LoadJoints_Array, LoadVectors_Array)
%
% compute forces in bars and reaction forces
%
% input:  Joints_Array             - coordinates of joints
%         MemberConnectivity_Array - connectivity
%         ReactionJoints_Array     - joint id where reaction acts on
%         ReactionVector_Array     - unit vector associated with reaction force
%         LoadJoints_Array         - joint id where external load acts on
%         LoadVectors_Array        - load vector
%
% output: barforces    - force magnitude in bars
%         reacforces   - reaction forces

    % extract number of joints, bars, reactions, and loads
    numjoints = size(Joints_Array,             1);
    numbars   = size(MemberConnectivity_Array, 1);
    numreact  = size(ReactionJoints_Array,     1);
    numloads  = size(LoadJoints_Array,         1);

    % number of equations
    numeqns = 3 * numjoints;

    % Defines bar length per unit mass [kg/m]:
    LengthPerUnitMass = .0953;

    % allocate arrays for linear system
    Amat = zeros(numeqns);
    bvec = zeros(numeqns, 1);

    % build Amat - loop over all joints

    for i = 1:numjoints

        % equation id numbers
        idx = 3*i-2;
        idy = 3*i-1;
        idz = 3*i; %change this

        % get all bars connected to joint
        [ibar, ijt] = find(MemberConnectivity_Array == i);

        % loop over all bars connected to joint
        for ib = 1:length(ibar)

            % get bar id
            barid = ibar(ib);

            % get coordinates for joints "i" and "j" of bar "barid"
            joint_i = Joints_Array(i,:);
            if ijt(ib) == 1
                jid = MemberConnectivity_Array(barid,2);
            else
                jid = MemberConnectivity_Array(barid,1);
            end
            joint_j = Joints_Array(jid,:);

            % compute unit vector pointing away from joint i
            vec_ij = joint_j - joint_i;
            uvec   = vec_ij / norm(vec_ij);

            % add unit vector into Amat
            Amat([idx idy idz], barid) = uvec; % added idz
        end
    end

    % build contribution of support reactions
    for i = 1:numreact

        % get joint id at which reaction force acts
        jid = ReactionJoints_Array(i);

        % equation id numbers
        idx = 3*jid-2;
        idy = 3*jid-1;
        idz = 3*jid-0; % added this

        % add unit vector into Amat
        Amat([idx idy idz], numbars + i) = ReactionVector_Array(i,:); %added idz
    end

    % Resizes array for weight:
     LoadVectors_Array(3:6, 1:3) = 0;

    % Calclates bar weight:
    for i = 1:numbars
        % Gets bar connectivity:
        JointA = MemberConnectivity_Array(i,1);
        JointB = MemberConnectivity_Array(i,2);

        % Determines length of bar:
        BarLength = [sum((Joints_Array(JointA,1:3) - ...
            Joints_Array(JointB,1:3)).^2)];

        % Calculates weight of bar:
        BarWeight(i) = BarLength * LengthPerUnitMass * 9.81;

    end

    % Includes bar weight as an external force:
    % * Adds force evenly on the two joints connecting bar *
    for i = 1:numbars

        % Gets weight of bar:
          Weight = -1 * BarWeight(i);

        % Determines joint to place the half the weight force to z comp:
        switch MemberConnectivity_Array(i,1)
            case 1
                LoadVectors_Array(1,3) = (Weight / 2) + LoadVectors_Array(1,3);
            case 2
                LoadVectors_Array(2,3) = (Weight / 2) + LoadVectors_Array(2,3);
            case 3
                LoadVectors_Array(3,3) = (Weight / 2) + LoadVectors_Array(3,3);
            case 4
                LoadVectors_Array(4,3) = (Weight / 2) + LoadVectors_Array(4,3);
            case 5
                LoadVectors_Array(5,3) = (Weight / 2) + LoadVectors_Array(5,3);
            case 6
                LoadVectors_Array(6,3) = (Weight / 2) + LoadVectors_Array(6,3);
        end

         % Determines joint to place the half the weight force to z comp:
        switch MemberConnectivity_Array(i,2)
            case 1
                LoadVectors_Array(1,3) = (Weight / 2) + LoadVectors_Array(1,3);
            case 2
                LoadVectors_Array(2,3) = (Weight / 2) + LoadVectors_Array(2,3);
            case 3
                LoadVectors_Array(3,3) = (Weight / 2) + LoadVectors_Array(3,3);
            case 4
                LoadVectors_Array(4,3) = (Weight / 2) + LoadVectors_Array(4,3);
            case 5
                LoadVectors_Array(5,3) = (Weight / 2) + LoadVectors_Array(5,3);
            case 6
                LoadVectors_Array(6,3) = (Weight / 2) + LoadVectors_Array(6,3);
        end

    end

    % Updates to inlude weight force acting on joints:
    LoadJoints_Array(1:6,1) = 1:6;

    % build load vector
    for i = 1:numloads

        % get joint id at which external force acts
        jid = LoadJoints_Array(i);

        % equation id numbers
        idx = 3*jid-2;
        idy = 3*jid-1;
        idz = 3*jid-0; %added this

        % add unit vector into bvec (sign change)
        bvec([idx idy idz]) = -1 * LoadVectors_Array(i,:); %added idz
    end

    % check for invertability of Amat
    if rank(Amat) ~= numeqns
        % disp(Amat);
        error('Amat is rank defficient: %d < %d\n', rank(Amat), numeqns);
    end

    % solve system
    xvec = Amat \ bvec;

    % extract forces in bars and reaction forces
    barforces  = xvec(1:numbars);
    reacforces = xvec((numbars + 1):end);

end
