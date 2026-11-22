%% export_drone_stl.m
% Export the SolidWorks drone (SimpleDrone.SLDASM) to STL through Autodesk
% Inventor, so MATLAB can draw the real CAD shape in animate_drone_3d.m.
%
% Output (folder .\stl):
%   <part>.stl         geometry of each component, in its own part frame
%   drone_parts.txt    one line per component: name | stl file | R (3x3, row-wise) | t (m) | box (m)
%
% Run once:  >> export_drone_stl
% Needs Autodesk Inventor on this PC (it opens .SLDPRT / .SLDASM files directly).

d = fileparts(mfilename('fullpath'));
if isempty(d), d = pwd; end
outDir = fullfile(d, 'stl');
if ~exist(outDir, 'dir'), mkdir(outDir); end

inv = actxserver('Inventor.Application');
inv.Visible = true;
inv.SilentOperation = true;          % no import dialogs

asmPath = fullfile(d, 'SimpleDrone.SLDASM');
asm = invoke(inv.Documents, 'Open', asmPath, true);
occs = asm.ComponentDefinition.Occurrences;

fid = fopen(fullfile(outDir, 'drone_parts.txt'), 'w');
done = containers.Map();
nOcc = occs.Count;
for k = 1:nOcc
    occ = invoke(occs, 'Item', k);
    if occ.Suppressed, continue; end
    nm  = occ.Name;                                   % e.g. "Propeller:1"
    pdoc = occ.Definition.Document;
    [~, base] = fileparts(pdoc.FullFileName);
    base = regexprep(base, '[^\w]', '_');
    stlFile = fullfile(outDir, [base '.stl']);

    if ~isKey(done, base)
        invoke(pdoc, 'SaveAs', stlFile, true);        % .stl extension -> STL translator
        rb = pdoc.ComponentDefinition.RangeBox;       % Inventor internal unit = cm
        box = [rb.MinPoint.X rb.MinPoint.Y rb.MinPoint.Z rb.MaxPoint.X rb.MaxPoint.Y rb.MaxPoint.Z]/100;
        done(base) = box;
    end
    box = done(base);

    T = occ.Transformation;                           % 4x4, translation in cm
    M = zeros(4);
    for r = 1:4
        for c = 1:4
            M(r, c) = invoke(T, 'Cell', r, c);
        end
    end
    R = M(1:3, 1:3);
    t = M(1:3, 4)'/100;                               % cm -> m

    fprintf(fid, '%s | %s.stl | %s | %s | %s\n', nm, base, ...
        num2str(reshape(R', 1, 9), '%.6f '), num2str(t, '%.6f '), num2str(box, '%.6f '));
end
fclose(fid);
invoke(asm, 'Close', true);
fprintf('Exported %d components to %s\n', nOcc, outDir);
type(fullfile(outDir, 'drone_parts.txt'));
