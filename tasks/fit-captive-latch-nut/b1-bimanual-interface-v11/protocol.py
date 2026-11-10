"""Portable source/evidence contract. A local ROOT review is not a signature service."""
from pathlib import Path
import json,hashlib,math
from strict_json import dumps
O=Path(__file__).resolve().parent
CHANNELS={'clock','joint_geometry','tool_nut_torque','robot_tool_contacts','wrist_force','nonfinger_contacts','passive_release','damage','drive_caps','reset'}
# Immutable manifest of the exact uncredited physical v9 replay. Its launcher,
# result and journal are never rewritten to resemble the verifier derivative.
ORIGINAL_V9_MANIFEST=__import__('json').loads('{"files":{"CONTROLLER-DERIVATION.json":"4255960a958b39a1a22471152697c49bf16c5987d4262270ae053b8bbd1b64dc","GPU_REQUEST_UNSUBMITTED.sbatch":"f68c20e04324fab29c0902a9770630ebda06eaebc7cb55fcaa04346791811241","OPERATOR.md":"b39e9c079bba153bae7d409d95eabe9502e3970c7c1c39ab13c40f564b75c023","README.md":"7647c470217c21c9a3b34c0bc04ea99073bb347b3774c0f3a3300c76eb2774b9","candidate/NOTICE.md":"5102f8f4cd258055fa2f2c55d191fe900d1a719acac6c5e5c550711fa2c1b020","candidate/README.md":"98e657e1a5869240d874394ef045a84545c05505be17a22a1dc69da221aefd17","candidate/assets/fixture.usda":"4ab72912ec20c638c903abb22bc4e5a31ebe5aafd8bd08e81eadcf95ef21ee94","candidate/assets/source-geometry.xml":"aaa1d2040f1b346a8f7ed65abdd04d24e49111f42f15cf17a026517844959325","candidate/assets/w12.usda":"5b7853f91c33023ee04b0fd555e382fdb1aa9d5295e80ca27ed6ccc2b1fdefac","candidate/assets/w13.usda":"11a109865245dd20379ca46beb77a5d90050600eefdae3a3fd093e9cc83f05bb","candidate/assets/w14.usda":"e16feb1db231a2aaf6b6c8a8713a282ca1cd872f6d26b14ccf4bed08cd2545a1","candidate/assets/w1_2.usda":"8969c92b8d8512c69b143668b14fd314951d658e35a4f5c5843b7dc52f477461","candidate/assets/wadj.usda":"b87422c826f45a6ee0aa30c8291cfb8929558eebdee599dde534995f47884b6d","candidate/build_usd.py":"0eb6dd0ee8fa245f88ac96bf28e1d3586ac28ce4e4e1f6041b75356ae6d63d2f","candidate/candidate-source.json":"f940eaf4302d35577e080f60b8d3b3c6dd754a542eca4bffde01dd5fb4fe8a90","candidate/contacts.py":"3fc7c5f73931ac2d8726a9428aaa14078f88a60a36b0ad3286d93251fb37ac15","candidate/dependency-versions.json":"92a04c2b6c7d81ced98a5275c9b2ed6f1f7500510ece878360c6a10544773787","candidate/evaluator.py":"f3e09b4d66af993c7fb477625d279c6a11133415dc1c098d5a3f2c5b2bc3b916","candidate/extract_geometry.py":"65265241c5cf94cd12ebe74aecf3f1a5bbcf4322f1d1ad897adb330dc6803805","candidate/fixture.json":"dacd07d5c3be4544101d1cb3ecf8b38804a19bf28a0727cb4941c9e89e0bb05f","candidate/foundation.py":"e20dee757942ea90784a07472b63355f1c194ecda973507d69923931e1f46533","candidate/history.py":"e453060722a776f3c2acf129dcab3b45c0cf361347cb8661970f44ba1f4e23fa","candidate/kit.py":"8c39c92ec52ef6384513541873c66e54596a8bd76ad0b0a55848bdf66b85a61f","candidate/motor_limits.py":"af6eb33db21889e24cae3467d98b3f0cb82cd2dca9dadf5d83e69f7181757a8b","candidate/physical-inventory.json":"2b4eb08ac84470fa726562fa2757a688d343b6365ef665da8268fda3a5d7e934","candidate/physics.py":"f7d56a036e12b39c4263baa99633ce5cb93e18bfbaaaa95ab15260a43eefb826","candidate/preflight_ik.py":"397e0960cd41b1dcbee1bd37afeabef4dc4e6748f84cf524841a4013ab6ab8bc","candidate/provenance/ASSETS.md":"4229ed36f9686862ada42e6e79160ff8568b7788051de3ef7727cff5e9f5cae4","candidate/provenance/B1_wrench.py":"02ec8984f41a02dcfc1220cbd5e4ad6caf7bb34e1f83e0a00dba7d6d2520c5a5","candidate/provenance/NATIVE_DEVIATIONS.md":"5114dd7b16ea9c0cbec4350e5f783bf9052b6fd09bf5fef0f1b2805968be8463","candidate/provenance/PROVENANCE.json":"d41be7508349182a1ce648162b3d2ac84605ebdbf24cb1553280f6c528f2bea6","candidate/provenance/YAM_LICENSE.i2rt":"7cd0a0224a287ebd6c45703eb6623d264c7810f8958d65de8f99e87ccd9f4891","candidate/provenance/YAM_SOURCE_COMMIT":"d201031f2bc4f6f41434b011c49082c459c43e9125c1b2f95099a0b9ce7dadb1","candidate/provenance/native-sourcehashes.json":"bc16e3ebefe74c28c9724536406ae1e6dc0a91db98135b8c9892d4d13d2f1389","candidate/provenance/w1_scoring.py":"fa7c18e40d49ab1d105a8ac4c35a3c71cb8542f7a2710935d82bede39cceb5ba","candidate/reference.py":"b871fcfdbc985de829031858d9d289361611427b1fd1951ff30f5eb7c2870419","candidate/source_rubric.py":"50238d39126d0b1a2adfb8dba66a34e360b8ab9f10f6276c970e87b937c47190","candidate/task.draft.json":"91ec388163fc0cbebdab52471bf0828b95750c3a71139349aebdb61f7d20d9d6","candidate/task.py":"e9ec59feee73196ce55565df50ca36e0550b81e310bcd4d8f268b7e509874773","candidate/verify_assets.py":"b6fd9fc114496ed6b02be606fa412ac3aaee0dadbb782d42ad55bc777fdeb021","candidate/verify_cpu.py":"45dc90414724f1826d04e238965b822bec808ca5605f09993d7f9b0d4a1598d5","full-consumed-source-map.json":"ff5813698d311a2015c3b696d0765dd6053157142c183743ec7e4caa9414b984","model_child.py":"50c8c53016b72db0f90ae5ac961c6a925bf736de524448e03d4deadd6b36f360","model_task.py":"c4ac393a51a6caa8a2239fa4f308ab434da91b37bd4ab16544a44f9ae57d6e90","observer.py":"3644146d9776d75a8b66e505e744fba447a9ca86aee521fc719092d75ba89508","pre_sdk.py":"876bb0e60a4dcae7e9fb9df6fff7bdbafd41859ff4d374a535d5172f2c2f673b","protocol.py":"caefa8c35ff590e359839c4abe3004e0ed32c3881ffd4241c1f58fd3dc840ffd","provenance/actual43839-fit-result.json":"7fa1502d4aebcc02002f6cf532563e412b77875a2c5f45fce0a0b38f3c8767e2","provenance/actual43839-launcher-result.json":"f6a49ea97e929c2116f2818200cfea1b2f89d690132511a8aa7e1ac90701cee7","provenance/asset-input-sha256.json":"072138bb89069992b08c4fd906cd15a3f778ec7604e403629417f59740be9a51","provenance/conditional-result.json":"f241567bfef91927aa8b80d4ca775a9a222ddc48aae28a4130d50f2725c5f8d2","provenance/consumed-source-sha256.json":"e94e8836fa549e0544a771d9783354151a25856ecc1ffede696d19772ff2f419","provenance/cpu-consumed-source.json":"9dbc8de4e8d4621ab42eebfcc23fc4820c5065f999e2b71b79f6413552d16598","provenance/finite-continuation.json":"839c9f0961157e24996ff5aec968cd4a6cb91ed20822840edf9ceccfdb8bf586","provenance/geometry-consumed-source.json":"5cc922b3110156802ef6e46e4fbdbd7fdc07c78300fb29088236ccf43e71de4d","provenance/manifest.json":"c766d1389ce9fa407a3843f0d77d42b78d99699149d33834f6f9dfb4b1381541","provenance/measured-grasp-drift.json":"a7a2cfcc26e8e951b6d433711a6a1cb495c7de984754038015d811c6601eff30","provenance/native-exact.json":"1358df2e6408a70cc5c051e8400d38ea02b0dbba12cf25c345663d65193af47e","provenance/original-v8-candidate-source.json":"5e0f33995673119cefc3ee51598724de737c05b1445a7f9c5a45a312e1955247","provenance/original-v8-native-exact.json":"f8fac2ed99aff0fd64bded63379843d764dd4d8ff51bc2fca06399bf9288b206","provenance/original-v8-reference.py":"4fd240d7f48ebf036a424183421f210260129be48c94a017283dc050de07d797","provenance/review.json":"521581b8ac42963731f25212976bea716d5880b86a361914692cd1ac5868e349","provenance/root-43852-source-receipt.json":"5a428df58a89356d23ddd67bc40d8c254de5dca947cc9d0a435dea482a692cda","receipt_builder.py":"01a2466c071f0ef84d422aabbc5186e55a0e407299fa93bca8b47c4af95f6be4","run_phase.py":"2932ee184ce0c4eba23dea296628864fcb096658a255e2f534f16bc34424997c","runtime-source-pins.json":"f2932f4b378810a71faa8b33bbe14b250dbb8468a2df2f564470d13ab648a567","runtime.py":"0e61bc77b5f29b1e036776f9598ddcaaa359e3d89118934f715f4a420f18624b","seal.py":"78bdf4d187ca8235bc0f9f7b1b65f1263ca811649f49a4ff2ba2d85833d11872","strict_json.py":"0f57b07aa9918c0b0c37677e7711c3681f087cfc729feb9737e098710ba45209","test_model_boundary.py":"435fc238752ea93d35ba6a7948d5f380366dbe411a36568fe59ca419c95b448d","test_protocol.py":"50029852b428f42b20f8e6e9dc5bbb49be25a9c3a39f85a8cbd4bb0c12b6bcbb","verify_model.py":"03bca744f7b37445d2ffd2fdcac8321bf7ccbb97a1fe01bf431170e2002fda4f","verify_runtime.py":"c18c9ed263a5bbc96fed69948e2519a6abe01b553f039d5bccc8a5f7ac27d2b6","video_capture.py":"845dda050f0185a8770935972679b895c849021a2d982da365b8abe4039c3d98"},"fingerprint":"99a55409d4d47c06ec7da177eeac1a37336c532ee82bb6a285d8464aab6c716b","native_manifest_sha256":"f940eaf4302d35577e080f60b8d3b3c6dd754a542eca4bffde01dd5fb4fe8a90","physical_proof":false,"schema":"b1-v8-portable"}')
ORIGINAL_V10_MANIFEST=__import__('json').loads('{"files":{"CONTROLLER-DERIVATION.json":"4255960a958b39a1a22471152697c49bf16c5987d4262270ae053b8bbd1b64dc","GPU_REQUEST_UNSUBMITTED.sbatch":"f68c20e04324fab29c0902a9770630ebda06eaebc7cb55fcaa04346791811241","OPERATOR.md":"b39e9c079bba153bae7d409d95eabe9502e3970c7c1c39ab13c40f564b75c023","README.md":"7647c470217c21c9a3b34c0bc04ea99073bb347b3774c0f3a3300c76eb2774b9","candidate/NOTICE.md":"5102f8f4cd258055fa2f2c55d191fe900d1a719acac6c5e5c550711fa2c1b020","candidate/README.md":"98e657e1a5869240d874394ef045a84545c05505be17a22a1dc69da221aefd17","candidate/assets/fixture.usda":"4ab72912ec20c638c903abb22bc4e5a31ebe5aafd8bd08e81eadcf95ef21ee94","candidate/assets/source-geometry.xml":"aaa1d2040f1b346a8f7ed65abdd04d24e49111f42f15cf17a026517844959325","candidate/assets/w12.usda":"5b7853f91c33023ee04b0fd555e382fdb1aa9d5295e80ca27ed6ccc2b1fdefac","candidate/assets/w13.usda":"11a109865245dd20379ca46beb77a5d90050600eefdae3a3fd093e9cc83f05bb","candidate/assets/w14.usda":"e16feb1db231a2aaf6b6c8a8713a282ca1cd872f6d26b14ccf4bed08cd2545a1","candidate/assets/w1_2.usda":"8969c92b8d8512c69b143668b14fd314951d658e35a4f5c5843b7dc52f477461","candidate/assets/wadj.usda":"b87422c826f45a6ee0aa30c8291cfb8929558eebdee599dde534995f47884b6d","candidate/build_usd.py":"0eb6dd0ee8fa245f88ac96bf28e1d3586ac28ce4e4e1f6041b75356ae6d63d2f","candidate/candidate-source.json":"f940eaf4302d35577e080f60b8d3b3c6dd754a542eca4bffde01dd5fb4fe8a90","candidate/contacts.py":"3fc7c5f73931ac2d8726a9428aaa14078f88a60a36b0ad3286d93251fb37ac15","candidate/dependency-versions.json":"92a04c2b6c7d81ced98a5275c9b2ed6f1f7500510ece878360c6a10544773787","candidate/evaluator.py":"f3e09b4d66af993c7fb477625d279c6a11133415dc1c098d5a3f2c5b2bc3b916","candidate/extract_geometry.py":"65265241c5cf94cd12ebe74aecf3f1a5bbcf4322f1d1ad897adb330dc6803805","candidate/fixture.json":"dacd07d5c3be4544101d1cb3ecf8b38804a19bf28a0727cb4941c9e89e0bb05f","candidate/foundation.py":"e20dee757942ea90784a07472b63355f1c194ecda973507d69923931e1f46533","candidate/history.py":"e453060722a776f3c2acf129dcab3b45c0cf361347cb8661970f44ba1f4e23fa","candidate/kit.py":"8c39c92ec52ef6384513541873c66e54596a8bd76ad0b0a55848bdf66b85a61f","candidate/motor_limits.py":"af6eb33db21889e24cae3467d98b3f0cb82cd2dca9dadf5d83e69f7181757a8b","candidate/physical-inventory.json":"2b4eb08ac84470fa726562fa2757a688d343b6365ef665da8268fda3a5d7e934","candidate/physics.py":"f7d56a036e12b39c4263baa99633ce5cb93e18bfbaaaa95ab15260a43eefb826","candidate/preflight_ik.py":"397e0960cd41b1dcbee1bd37afeabef4dc4e6748f84cf524841a4013ab6ab8bc","candidate/provenance/ASSETS.md":"4229ed36f9686862ada42e6e79160ff8568b7788051de3ef7727cff5e9f5cae4","candidate/provenance/B1_wrench.py":"02ec8984f41a02dcfc1220cbd5e4ad6caf7bb34e1f83e0a00dba7d6d2520c5a5","candidate/provenance/NATIVE_DEVIATIONS.md":"5114dd7b16ea9c0cbec4350e5f783bf9052b6fd09bf5fef0f1b2805968be8463","candidate/provenance/PROVENANCE.json":"d41be7508349182a1ce648162b3d2ac84605ebdbf24cb1553280f6c528f2bea6","candidate/provenance/YAM_LICENSE.i2rt":"7cd0a0224a287ebd6c45703eb6623d264c7810f8958d65de8f99e87ccd9f4891","candidate/provenance/YAM_SOURCE_COMMIT":"d201031f2bc4f6f41434b011c49082c459c43e9125c1b2f95099a0b9ce7dadb1","candidate/provenance/native-sourcehashes.json":"bc16e3ebefe74c28c9724536406ae1e6dc0a91db98135b8c9892d4d13d2f1389","candidate/provenance/w1_scoring.py":"fa7c18e40d49ab1d105a8ac4c35a3c71cb8542f7a2710935d82bede39cceb5ba","candidate/reference.py":"b871fcfdbc985de829031858d9d289361611427b1fd1951ff30f5eb7c2870419","candidate/source_rubric.py":"50238d39126d0b1a2adfb8dba66a34e360b8ab9f10f6276c970e87b937c47190","candidate/task.draft.json":"91ec388163fc0cbebdab52471bf0828b95750c3a71139349aebdb61f7d20d9d6","candidate/task.py":"e9ec59feee73196ce55565df50ca36e0550b81e310bcd4d8f268b7e509874773","candidate/verify_assets.py":"b6fd9fc114496ed6b02be606fa412ac3aaee0dadbb782d42ad55bc777fdeb021","candidate/verify_cpu.py":"45dc90414724f1826d04e238965b822bec808ca5605f09993d7f9b0d4a1598d5","full-consumed-source-map.json":"ff5813698d311a2015c3b696d0765dd6053157142c183743ec7e4caa9414b984","model_child.py":"50c8c53016b72db0f90ae5ac961c6a925bf736de524448e03d4deadd6b36f360","model_task.py":"c4ac393a51a6caa8a2239fa4f308ab434da91b37bd4ab16544a44f9ae57d6e90","observer.py":"3644146d9776d75a8b66e505e744fba447a9ca86aee521fc719092d75ba89508","pre_sdk.py":"876bb0e60a4dcae7e9fb9df6fff7bdbafd41859ff4d374a535d5172f2c2f673b","protocol.py":"69eabec3bbe5f20f3449f5b01de11944e7bf4f2b6dd332ca92403283657080ad","provenance/actual43839-fit-result.json":"7fa1502d4aebcc02002f6cf532563e412b77875a2c5f45fce0a0b38f3c8767e2","provenance/actual43839-launcher-result.json":"f6a49ea97e929c2116f2818200cfea1b2f89d690132511a8aa7e1ac90701cee7","provenance/asset-input-sha256.json":"072138bb89069992b08c4fd906cd15a3f778ec7604e403629417f59740be9a51","provenance/conditional-result.json":"f241567bfef91927aa8b80d4ca775a9a222ddc48aae28a4130d50f2725c5f8d2","provenance/consumed-source-sha256.json":"e94e8836fa549e0544a771d9783354151a25856ecc1ffede696d19772ff2f419","provenance/cpu-consumed-source.json":"9dbc8de4e8d4621ab42eebfcc23fc4820c5065f999e2b71b79f6413552d16598","provenance/finite-continuation.json":"839c9f0961157e24996ff5aec968cd4a6cb91ed20822840edf9ceccfdb8bf586","provenance/geometry-consumed-source.json":"5cc922b3110156802ef6e46e4fbdbd7fdc07c78300fb29088236ccf43e71de4d","provenance/manifest.json":"c766d1389ce9fa407a3843f0d77d42b78d99699149d33834f6f9dfb4b1381541","provenance/measured-grasp-drift.json":"a7a2cfcc26e8e951b6d433711a6a1cb495c7de984754038015d811c6601eff30","provenance/native-exact.json":"1358df2e6408a70cc5c051e8400d38ea02b0dbba12cf25c345663d65193af47e","provenance/original-v8-candidate-source.json":"5e0f33995673119cefc3ee51598724de737c05b1445a7f9c5a45a312e1955247","provenance/original-v8-native-exact.json":"f8fac2ed99aff0fd64bded63379843d764dd4d8ff51bc2fca06399bf9288b206","provenance/original-v8-reference.py":"4fd240d7f48ebf036a424183421f210260129be48c94a017283dc050de07d797","provenance/review.json":"521581b8ac42963731f25212976bea716d5880b86a361914692cd1ac5868e349","provenance/root-43852-source-receipt.json":"5a428df58a89356d23ddd67bc40d8c254de5dca947cc9d0a435dea482a692cda","receipt_builder.py":"01a2466c071f0ef84d422aabbc5186e55a0e407299fa93bca8b47c4af95f6be4","run_phase.py":"2932ee184ce0c4eba23dea296628864fcb096658a255e2f534f16bc34424997c","runtime-source-pins.json":"f2932f4b378810a71faa8b33bbe14b250dbb8468a2df2f564470d13ab648a567","runtime.py":"0e61bc77b5f29b1e036776f9598ddcaaa359e3d89118934f715f4a420f18624b","seal.py":"78bdf4d187ca8235bc0f9f7b1b65f1263ca811649f49a4ff2ba2d85833d11872","strict_json.py":"0f57b07aa9918c0b0c37677e7711c3681f087cfc729feb9737e098710ba45209","test_model_boundary.py":"435fc238752ea93d35ba6a7948d5f380366dbe411a36568fe59ca419c95b448d","test_protocol.py":"50029852b428f42b20f8e6e9dc5bbb49be25a9c3a39f85a8cbd4bb0c12b6bcbb","verify_model.py":"03bca744f7b37445d2ffd2fdcac8321bf7ccbb97a1fe01bf431170e2002fda4f","verify_runtime.py":"c18c9ed263a5bbc96fed69948e2519a6abe01b553f039d5bccc8a5f7ac27d2b6","video_capture.py":"845dda050f0185a8770935972679b895c849021a2d982da365b8abe4039c3d98"},"fingerprint":"e46f7b0c845dd87dd556afe4f289e1b09098f63d94970875db005903ec8da833","native_manifest_sha256":"f940eaf4302d35577e080f60b8d3b3c6dd754a542eca4bffde01dd5fb4fe8a90","physical_proof":false,"schema":"b1-v8-portable"}')
INTERFACE_V11_HASHES={'candidate/kit.py': 'd470e62b49261273e465f2f8dda3c990fd1fb61d18f15dbbd3ec164f65b3293b', 'candidate/candidate-source.json': '9c256b589ce6770452291466fea16532d89eeeab40799c4d5a2eb94ea39ab8cd', 'provenance/native-exact.json': '68835a26263fef5623fe8e34b56606ca32f6205b3e2e9d65d35428806bb6f2cc'}
def matching_replay_manifest(current,recorded):
 if recorded==current:return recorded
 if recorded not in (ORIGINAL_V9_MANIFEST,ORIGINAL_V10_MANIFEST):raise ValueError('unknown historical replay source')
 if set(current)!=set(recorded) or set(current['files'])!=set(recorded['files']):raise ValueError('historical runtime inventory changed')
 allowed={'protocol.py','candidate/kit.py','candidate/candidate-source.json','provenance/native-exact.json'}
 if {n for n in current['files'] if current['files'][n]!=recorded['files'][n]}!=allowed:raise ValueError('historical runtime differs beyond declared reset/interface metadata')
 for name,value in INTERFACE_V11_HASHES.items():
  if current['files'][name]!=value:raise ValueError('unrecognized interface derivative')
 for key in current:
  if key not in ('files','fingerprint','native_manifest_sha256') and current[key]!=recorded[key]:raise ValueError('historical manifest metadata changed')
 if current['native_manifest_sha256']!=INTERFACE_V11_HASHES['candidate/candidate-source.json']:raise ValueError('native manifest bridge changed')
 return recorded

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loads(s):
 def pairs(xs):
  d={}
  for k,v in xs:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 def finite_float(x):
  v=float(x)
  if not math.isfinite(v):raise ValueError('nonfinite JSON float')
  return v
 return json.loads(s,object_pairs_hook=pairs,parse_float=finite_float,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite JSON')))
def load(p):return loads(Path(p).read_text())
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('strict finite number required')
 return v
def relative(base,n):
 if type(n) is not str or not n or Path(n).is_absolute():raise ValueError('relative file required')
 p=(Path(base)/n).resolve()
 if not p.is_relative_to(Path(base).resolve()) or not p.is_file():raise ValueError('missing/escaped file')
 return p
def check():
 m=load(O/'manifest.json')
 expected=hashlib.sha256(dumps(m['files'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
 if m['fingerprint']!=expected:raise ValueError('fingerprint mismatch')
 for n,h in m['files'].items():
  if sha(relative(O,n))!=h:raise ValueError('source mismatch '+n)
 for n,h in load(O/'provenance/native-exact.json').items():
  if sha(relative(O/'candidate',n))!=h:raise ValueError('native/core changed '+n)
 return m

def fresh(folder):
 m=check();f=Path(folder).resolve();launch=load(f/'launcher.json');result=load(f/'result.json')
 import runpy
 rubric=runpy.run_path(str(O/'candidate/provenance/w1_scoring.py'))
 if launch.get('phase')!='replay' or type(launch.get('exitcode')) is not int or launch['exitcode']!=0:raise ValueError('actual clean replay required')
 observed=matching_replay_manifest(m,launch.get('source_before'))
 if launch.get('source_after')!=observed:raise ValueError('fresh source mismatch')
 if result.get('kind')!='ACTUAL_B1_RUNTIME' or result.get('phase')!='replay' or result.get('completed') is not True or result.get('error') is not None or result.get('close_error') is not None:raise ValueError('actual completion absent')
 if result.get('manifest')!=observed or result.get('promoted_channels')!=[]:raise ValueError('replay must be uncredited/source exact')
 rows=[];historical_right=None
 with (f/'journal.jsonl').open() as stream:
  for line in stream:
   r=loads(line);s=r['status'];i=len(rows)
   if r.get('kind')!='ACTUAL_RUNTIME_OBSERVER' or type(s['controls']) is not int or s['controls']!=i:raise ValueError('control discontinuity')
   if type(s['physics_samples']) is not int or s['physics_samples']!=25*i or abs(number(s['physics_s'])-i*.05)>1e-6:raise ValueError('native clock mismatch')
   if number(s['score'])!=0 or s['missing_runtime_channels']!=sorted(CHANNELS):raise ValueError('unproved channels activated')
   # The reset image precedes the first genuine physics sample. It is an
   # explicitly unscored sentinel, not a scored veto or missing measured row.
   reset_sentinel=(i==0 and r.get('phase')=='reset' and type(r.get('controls')) is int and r.get('controls')==0 and r.get('action') is None and s.get('measurement') is None and s.get('tier')=='veto' and number(s.get('candidate_score'))==0 and s.get('gate')=='runtime_channels_unverified' and s.get('events')==[] and s.get('reasons')==['invalid measurement/history: measurement must be a dict'] and s['history']=={'rounded_any':False,'invalid_measurement_seen':False,'hold_full_s':0.0,'hold_release_s':0.0})
   if s['error'] is not None or (s['tier']=='veto' and not reset_sentinel) or s['history']['rounded_any'] is not False:raise ValueError('failed actual sample')
   if s.get('measurement') is None and not reset_sentinel:raise ValueError('missing actual measurement outside explicit reset sentinel')
   if number(r['physics_dt'])!=.002 or number(r['control_dt'])!=.05:raise ValueError('wrong clock')
   if i:
    if type(r['action']) is not list or len(r['action'])!=14:raise ValueError('14D action missing')
    for x in r['action']:number(x)
   if type(r['physics_tick']) is not int or (i and r['physics_tick']!=rows[0]['physics_tick']+25*i):raise ValueError('physical tick discontinuity')
   rubric['validate_history'](s['history'])
   if i and rubric['score'](s['measurement'],s['history'])['score']!=number(s['candidate_score']):raise ValueError('native scorer mismatch')
   if observed!=m and i:
    # Historical v9/v10 evidence is eligible only if removing the port-only
    # right-target validator was nonbinding throughout every recorded Action.
    right=[number(v) for v in r['action'][7:]]
    if historical_right is None:historical_right=right
    if any(abs(v-w)>1e-5 for v,w in zip(right,historical_right)):raise ValueError('historical right-target guard was binding')
   rows.append(r)
 if len(rows)<2:raise ValueError('actual journal absent')
 s=rows[-1]['status']
 if not 0<s['controls']<=3870 or number(s['candidate_score'])!=1 or number(s['history']['hold_full_s'])<20:raise ValueError('native full success/20s hold absent')
 if result.get('final_status')!=s:raise ValueError('terminal result mismatch')
 media=load(f/'media-finalization.json')
 if media.get('all_encoded') is not True or type(media.get('frames_per_camera')) is not int or media['frames_per_camera']!=len(rows):raise ValueError('actual frame continuity absent')
 if set(media.get('files',{}))!={'top_cam.mp4','left_cam.mp4','right_cam.mp4'}:raise ValueError('camera inventory')
 for n,row in media['files'].items():
  p=relative(f,n)
  if type(row['bytes']) is not int or row['bytes']!=p.stat().st_size or not 0<row['bytes']<=50_000_000:raise ValueError('media bytes')
 return {'controls':s['controls'],'hold_s':s['history']['hold_full_s'],'raw_score':s['candidate_score'],'public_score':s['score']}

def receipt(path):
 m=check();p=Path(path).resolve();r=load(p)
 if r.get('schema')!='b1-v8-root-physical-review' or r.get('approved') is not True or r.get('reviewer')!='ROOT' or r.get('manifest')!=m:raise ValueError('ROOT matching review required')
 if r.get('historical_fit545_is_full_success') is not False:raise ValueError('historical fit prefix cannot establish full success')
 if type(r.get('replay')) is not str or Path(r['replay']).is_absolute():raise ValueError('relative replay required')
 f=(p.parent/r['replay']).resolve()
 if not f.is_relative_to(p.parent):raise ValueError('replay escape')
 if fresh(f)!=r.get('actual_summary'):raise ValueError('stale actual summary')
 expected={str(x.relative_to(f)):sha(relative(f,str(x.relative_to(f)))) for x in f.rglob('*') if x.is_file()}
 if r.get('evidence_sha256')!=expected:raise ValueError('replay evidence changed')
 decisions=r.get('channels',{})
 if set(decisions)!=CHANNELS:raise ValueError('all native channel decisions required')
 for channel,row in decisions.items():
  if row.get('approved') is not True or not isinstance(row.get('rationale'),str) or not row['rationale'].strip():raise ValueError('unproved channel '+channel)
  proofs=row.get('proofs',[])
  if not proofs:raise ValueError('actual/source proof missing '+channel)
  for proof in proofs:
   if sha(relative(p.parent,proof['path']))!=proof['sha256']:raise ValueError('proof binding changed')
 if r.get('mounted_support_approved') is not True:raise ValueError('fresh mounted support review required')
 return r
