// opennuts -- the GMK CLI / server entry point.
//
// Usage:
//   opennuts                          show help
//   opennuts version                  print the kernel version and exit
//   opennuts lsp                      run the OpenNuts LSP over stdio (LSP framing)
//   opennuts geom-server [--lines]    run the geometry server over stdio
//                                     (default uses LSP framing; --lines
//                                     uses one JSON document per line)
//   opennuts parse <file.opennuts>    parse a source file and print
//                                     diagnostics to stderr (exit code = #errors)
//   opennuts check                    self-test: build a few primitives,
//                                     validate them, print bbox/face counts

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "gmk/brep/body.hpp"
#include "gmk/brep/primitives.hpp"
#include "gmk/brep/validate.hpp"
#include "gmk/lang/parser.hpp"
#include "gmk/server/geom_server.hpp"
#include "gmk/server/lsp_server.hpp"
#include "gmk/tess/tessellator.hpp"
#include "gmk/version.hpp"

using namespace gmk;

static int cmd_version() {
    std::cout << "opennuts (gmk) " << VERSION_STRING << "\n";
    return 0;
}

static int cmd_help() {
    std::cout <<
        "opennuts -- Geometric Modeling Kernel CLI\n"
        "\n"
        "Commands:\n"
        "  version                 Show kernel version.\n"
        "  lsp                     Run the OpenNuts LSP server on stdio.\n"
        "  geom-server [--lines]   Run the geometry server on stdio.\n"
        "  parse <file>            Parse an OpenNuts source and print diagnostics.\n"
        "  check                   Build a few primitives and validate them.\n"
        "\n"
        "Build " << VERSION_STRING << ".\n";
    return 0;
}

static int cmd_lsp() {
    lsp::LspServer s(std::cin, std::cout);
    return s.run() == Status::Ok ? 0 : 1;
}

static int cmd_geom_server(int argc, char** argv) {
    jsonrpc::Framing fr = jsonrpc::Framing::LspHeaders;
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], "--lines") == 0) fr = jsonrpc::Framing::NewlineDelimited;
    }
    geom_server::Server s(std::cin, std::cout, fr);
    return s.run() == Status::Ok ? 0 : 1;
}

static int cmd_parse(const char* path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "cannot open: " << path << "\n";
        return 2;
    }
    std::stringstream ss; ss << f.rdbuf();
    lang::Parser p(ss.str());
    p.parse();
    int errors = 0;
    for (const auto& d : p.diagnostics()) {
        std::cerr << path << ":" << d.pos.line << ":" << d.pos.column
                  << ": " << d.message;
        if (!d.code.empty()) std::cerr << " [" << d.code << "]";
        std::cerr << "\n";
        if (d.severity == lang::Diagnostic::Severity::Error) ++errors;
    }
    return errors;
}

static int cmd_check() {
    // Build a few primitives and validate them.
    auto run = [](const char* name, auto&& build) {
        brep::Body body;
        Status s = build(body);
        if (s != Status::Ok) {
            std::cout << name << ": construction failed (" << status_name(s) << ")\n";
            return 1;
        }
        brep::ValidationReport rep;
        brep::validate(body, rep);
        std::cout << name
                  << ": faces=" << body.face_count()
                  << " edges=" << body.edge_count()
                  << " verts=" << body.vertex_count()
                  << " errors=" << rep.error_count
                  << " warnings=" << rep.warning_count
                  << "\n";
        for (const auto& iss : rep.issues) {
            const char* sev = "info";
            switch (iss.severity) {
                case brep::ValidationIssue::Severity::Error:   sev = "ERROR";   break;
                case brep::ValidationIssue::Severity::Warning: sev = "warn";    break;
                case brep::ValidationIssue::Severity::Info:    sev = "info";    break;
            }
            std::cout << "  " << sev << ": " << iss.message << "\n";
        }
        // Tessellate as well.
        tess::TessOptions opts;
        tess::BodyMesh mesh;
        tess::tessellate(body, opts, mesh);
        tess::flatten(mesh);
        std::cout << "  mesh: vertices=" << mesh.flat_positions_m.size()
                  << " tris=" << (mesh.flat_triangles.size() / 3)
                  << "\n";
        return rep.error_count;
    };

    int rc = 0;
    rc += run("box(50,30,10)", [](brep::Body& b) {
        return brep::make_box(b, Vec3i{0,0,0},
                              mm_to_length(25), mm_to_length(15), mm_to_length(5));
    });
    rc += run("sphere(r=10mm)", [](brep::Body& b) {
        return brep::make_sphere(b, Vec3i{0,0,0}, mm_to_length(10));
    });
    rc += run("cylinder(r=5,h=20)", [](brep::Body& b) {
        return brep::make_cylinder(b, Vec3i{0,0,0}, Vec3d{0,0,1},
                                   mm_to_length(5), mm_to_length(20));
    });
    rc += run("cone(r_base=5,r_top=2,h=15)", [](brep::Body& b) {
        return brep::make_cone(b, Vec3i{0,0,0}, Vec3d{0,0,1},
                               mm_to_length(5), mm_to_length(2), mm_to_length(15));
    });
    rc += run("torus(R=10,r=2)", [](brep::Body& b) {
        return brep::make_torus(b, Vec3i{0,0,0}, Vec3d{0,0,1},
                                mm_to_length(10), mm_to_length(2));
    });

    std::cout << "check: total errors=" << rc << "\n";
    return rc;
}

int main(int argc, char** argv) {
    if (argc < 2)                                  return cmd_help();
    std::string cmd = argv[1];
    if (cmd == "-h" || cmd == "--help" || cmd == "help") return cmd_help();
    if (cmd == "version" || cmd == "--version")    return cmd_version();
    if (cmd == "lsp")                              return cmd_lsp();
    if (cmd == "geom-server")                      return cmd_geom_server(argc - 2, argv + 2);
    if (cmd == "parse") {
        if (argc < 3) { std::cerr << "usage: opennuts parse <file>\n"; return 2; }
        return cmd_parse(argv[2]);
    }
    if (cmd == "check")                            return cmd_check();
    std::cerr << "unknown command: " << cmd << "\n";
    cmd_help();
    return 2;
}
