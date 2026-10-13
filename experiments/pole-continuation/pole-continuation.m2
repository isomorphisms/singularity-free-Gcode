-- Exact offline limiting-C-setting diagnostic for the declared AC-head model.
-- Usage: M2 --script pole-continuation.m2 absolute/path/to/cases.tsv
-- The input describes two nonzero horizontal meridians approaching the lower
-- pole.  It does not describe full SO(3), a table, finite joint ranges, time,
-- collision geometry, compliance, or a controller command.

if #scriptCommandLine != 2 then error "FULTON-CNC-USAGE: M2 --script pole-continuation.m2 CASES_TSV";

inputPath = scriptCommandLine#1;
require = (condition, diagnostic) -> if not condition then error diagnostic;
rows = lines get inputPath;
require(#rows >= 2, "FULTON-CNC-INPUT-EMPTY");

header = separate("\\t", rows#0);
require(header == {"case_id","incoming_x","incoming_y","outgoing_x","outgoing_y","pose_kind"},
    "FULTON-CNC-HEADER");

validCaseID = caseID -> match("^[A-Za-z0-9][A-Za-z0-9._-]*$", caseID);
validRational = token -> match("^-?[0-9]+(/[1-9][0-9]*)?$", token);
parseRational = token -> (
    if not validRational token then error("FULTON-CNC-RATIONAL: " | token);
    value token
    );

-- The exceptional fiber itself is independent of a meridian.  For the
-- right-handed active Rz(C)Rx(A) head, lower-pole direction means
-- sinA=0 and cosA=1, leaving one reduced C-circle.
Q = QQ[poleSinA,poleCosA,poleSinC,poleCosC];
poleCarrier = ideal(poleSinA^2+poleCosA^2-1,poleSinC^2+poleCosC^2-1);
poleFiber = poleCarrier + ideal(poleSinA,poleCosA-1);
assert(dim(Q/poleFiber) == 1);
assert(radical poleFiber == poleFiber);
print("model=idealized_right_handed_active_Rz(C)Rx(A)_tip_plus_direction");
print("reduced_pole_freedom_dimension=" | toString dim(Q/poleFiber));
print("pole_radical_equals_original=" | toString(radical poleFiber == poleFiber));
print("pole_ideal_groebner=" | toString gens gb poleFiber);

R = QQ[sinC,cosC];
unitCircle = ideal(sinC^2+cosC^2-1);
caseIDs = {};

scan(drop(rows,1), line -> (
    fields = separate("\\t", line);
    require(#fields == 6, "FULTON-CNC-ROW-FIELDS: " | line);
    caseID = fields#0;
    require(validCaseID caseID, "FULTON-CNC-CASE-ID: " | caseID);
    if member(caseID,caseIDs) then error("FULTON-CNC-DUPLICATE-CASE: " | caseID);
    caseIDs = append(caseIDs,caseID);
    require(fields#5 == "tip-plus-direction", "FULTON-CNC-POSE-KIND: " | fields#5);

    incomingX = parseRational(fields#1);
    incomingY = parseRational(fields#2);
    outgoingX = parseRational(fields#3);
    outgoingY = parseRational(fields#4);
    if incomingX == 0 and incomingY == 0 then error("FULTON-CNC-ZERO-MERIDIAN: " | caseID | ":incoming");
    if outgoingX == 0 and outgoingY == 0 then error("FULTON-CNC-ZERO-MERIDIAN: " | caseID | ":outgoing");

    incomingLimit = unitCircle + ideal(incomingX*cosC+incomingY*sinC);
    outgoingLimit = unitCircle + ideal(outgoingX*cosC+outgoingY*sinC);
    intersectionScheme = incomingLimit + outgoingLimit;
    limitDeterminant = incomingX*outgoingY-incomingY*outgoingX;

    -- Every nonzero meridian line through the pole has a two-point reduced
    -- C-limit scheme.  The sum below is the scheme-theoretic intersection of
    -- the two particular limiting schemes, never the full pole circle.
    assert(dim(R/incomingLimit) == 0);
    assert(degree(R/incomingLimit) == 2);
    assert(radical incomingLimit == incomingLimit);
    assert(dim(R/outgoingLimit) == 0);
    assert(degree(R/outgoingLimit) == 2);
    assert(radical outgoingLimit == outgoingLimit);

    print("case_id=" | caseID);
    print("incoming_limit_ideal=(sinC^2+cosC^2-1," | toString incomingX | "*cosC+" | toString incomingY | "*sinC)");
    print("outgoing_limit_ideal=(sinC^2+cosC^2-1," | toString outgoingX | "*cosC+" | toString outgoingY | "*sinC)");
    print("limit_determinant=" | toString limitDeterminant);
    if limitDeterminant == 0 then (
        assert(dim(R/intersectionScheme) == 0);
        assert(degree(R/intersectionScheme) == 2);
        assert(radical intersectionScheme == intersectionScheme);
        print("intersection_ideal_groebner=" | toString gens gb intersectionScheme);
        print("intersection_scheme_dimension=0");
        print("intersection_scheme_degree=2");
        print("intersection_scheme_reduced=true");
        print("status=LIMIT_SETS_INTERSECT");
        ) else (
        assert(intersectionScheme == ideal(1_R));
        print("intersection_ideal_groebner=" | toString gens gb intersectionScheme);
        print("intersection_scheme=empty");
        print("status=LIMIT_SETS_DISJOINT");
        );
    ));

print("scope=limiting_C_settings_only;_not_clearance_collision_or_motion_authorization");
print("PASS pole-continuation");
exit 0;
