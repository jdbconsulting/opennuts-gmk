#pragma once
//
// Geometry server. A JSON-RPC service that creates, mutates and tessellates
// brep::Body instances on behalf of a client (typically a 3D display
// extension talking over stdio or a localhost socket).
//
// Methods (all params are objects unless noted):
//
//   gmk.version                              -> {version: string}
//   gmk.session.info                         -> {bodies: int, mesh_faces: int}
//   gmk.session.clear                        -> {ok: true}
//   gmk.body.delete       {id}               -> {ok: true}
//
//   gmk.primitive.box     {center:[x,y,z], hx, hy, hz, unit:"mm"}   -> {id}
//   gmk.primitive.sphere  {center:[x,y,z], r, unit:"mm"}            -> {id}
//   gmk.primitive.cylinder{base:[x,y,z], axis:[i,j,k], r, h, unit}  -> {id}
//   gmk.primitive.cone    {base, axis, r_base, r_top, h, unit}      -> {id}
//   gmk.primitive.torus   {center, axis, R, r, unit}                -> {id}
//
//   gmk.body.bbox         {id, unit?}        -> {min:[..], max:[..]}
//   gmk.body.validate     {id}               -> {issues:[{severity,message}]}
//
//   gmk.tessellate        {id, chord_tol_m?, unit?}
//                                            -> {positions:[...], normals:[...],
//                                                triangles:[...]}
//
//   gmk.lang.parse        {source}           -> {diagnostics:[...]}
//   gmk.lang.eval         {source, clear?}   -> {bodies:[{name,id}], diagnostics:[...]}
//
// Errors are returned via JSON-RPC's error mechanism with a small
// vocabulary of codes (see jsonrpc.hpp).
//

#include <cstdint>
#include <iosfwd>
#include <string>
#include <unordered_map>

#include "gmk/brep/body.hpp"
#include "gmk/lang/eval.hpp"
#include "gmk/result.hpp"
#include "gmk/server/json.hpp"
#include "gmk/server/jsonrpc.hpp"

namespace gmk::geom_server {

class Server : public lang::BodyStore {
public:
    Server(std::istream& in, std::ostream& out,
           jsonrpc::Framing framing = jsonrpc::Framing::LspHeaders);

    Status run();

    // Direct dispatch entry-point used by the WASM bridge and the test
    // harness. Returns a JSON response value or a JSON error envelope
    // suitable for sending back to the client.
    json::Value dispatch_request(const json::Value& req);

    // BodyStore interface (used by lang::eval_program).
    std::int64_t add(brep::Body body) override;

private:
    using BodyId = std::uint64_t;
    BodyId   next_id_ = 1;
    std::unordered_map<BodyId, brep::Body> bodies_;
    std::istream&    in_;
    std::ostream&    out_;
    jsonrpc::Framing framing_;

    // Method handlers. Each returns the bare "result" value or sets
    // ``out_err``+``out_err_msg`` to dispatch an error.
    json::Value handle(const std::string& method,
                       const json::Value* params,
                       int& out_err, std::string& out_err_msg);

    // Helpers shared across handlers.
    static length_t length_from_units(double value, const std::string& unit, bool& ok);
    static bool     vec3_from_array(const json::Value& v, length_t out[3],
                                    const std::string& unit);
    static bool     vec3d_from_array(const json::Value& v, double out[3]);
};

}  // namespace gmk::geom_server
