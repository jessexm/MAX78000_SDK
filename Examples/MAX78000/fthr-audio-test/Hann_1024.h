static const float32_t Hann_lut[1024] = {
    0.000000000f, 0.000009418f, 0.000037730f, 0.000084877f, 0.000150889f, 0.000235766f, 0.000339478f, 0.000462025f,
    0.000603437f, 0.000763714f, 0.000942796f, 0.001140684f, 0.001357406f, 0.001592964f, 0.001847297f, 0.002120435f,
    0.002412349f, 0.002723038f, 0.003052473f, 0.003400654f, 0.003767580f, 0.004153222f, 0.004557580f, 0.004980594f,
    0.005422324f, 0.005882680f, 0.006361693f, 0.006859303f, 0.007375538f, 0.007910341f, 0.008463740f, 0.009035647f,
    0.009626091f, 0.010235041f, 0.010862440f, 0.011508316f, 0.012172610f, 0.012855291f, 0.013556361f, 0.014275789f,
    0.015013546f, 0.015769571f, 0.016543895f, 0.017336428f, 0.018147171f, 0.018976122f, 0.019823194f, 0.020688385f,
    0.021571636f, 0.022472948f, 0.023392290f, 0.024329603f, 0.025284857f, 0.026258022f, 0.027249038f, 0.028257906f,
    0.029284567f, 0.030328989f, 0.031391114f, 0.032470942f, 0.033568382f, 0.034683436f, 0.035816044f, 0.036966145f,
    0.038133711f, 0.039318711f, 0.040521085f, 0.041740805f, 0.042977810f, 0.044232041f, 0.045503467f, 0.046792030f,
    0.048097730f, 0.049420446f, 0.050760150f, 0.052116811f, 0.053490371f, 0.054880768f, 0.056287974f, 0.057711899f,
    0.059152514f, 0.060609758f, 0.062083602f, 0.063573927f, 0.065080732f, 0.066603929f, 0.068143487f, 0.069699347f,
    0.071271449f, 0.072859675f, 0.074464053f, 0.076084495f, 0.077720881f, 0.079373240f, 0.081041455f, 0.082725465f,
    0.084425211f, 0.086140662f, 0.087871701f, 0.089618266f, 0.091380358f, 0.093157858f, 0.094950676f, 0.096758783f,
    0.098582119f, 0.100420594f, 0.102274120f, 0.104142666f, 0.106026143f, 0.107924491f, 0.109837592f, 0.111765474f,
    0.113707960f, 0.115665019f, 0.117636591f, 0.119622558f, 0.121622890f, 0.123637527f, 0.125666320f, 0.127709270f,
    0.129766226f, 0.131837159f, 0.133921981f, 0.136020601f, 0.138132989f, 0.140259027f, 0.142398596f, 0.144551665f,
    0.146718174f, 0.148897976f, 0.151091039f, 0.153297246f, 0.155516565f, 0.157748848f, 0.159994036f, 0.162252069f,
    0.164522856f, 0.166806251f, 0.169102281f, 0.171410739f, 0.173731625f, 0.176064819f, 0.178410202f, 0.180767745f,
    0.183137327f, 0.185518831f, 0.187912226f, 0.190317392f, 0.192734241f, 0.195162684f, 0.197602630f, 0.200053960f,
    0.202516615f, 0.204990506f, 0.207475513f, 0.209971577f, 0.212478548f, 0.214996397f, 0.217524946f, 0.220064193f,
    0.222614020f, 0.225174248f, 0.227744877f, 0.230325758f, 0.232916862f, 0.235518008f, 0.238129109f, 0.240750164f,
    0.243380934f, 0.246021390f, 0.248671383f, 0.251330972f, 0.253999829f, 0.256678045f, 0.259365439f, 0.262061864f,
    0.264767289f, 0.267481565f, 0.270204604f, 0.272936344f, 0.275676668f, 0.278425425f, 0.281182528f, 0.283947945f,
    0.286721408f, 0.289502978f, 0.292292535f, 0.295089841f, 0.297894925f, 0.300707638f, 0.303527892f, 0.306355447f,
    0.309190363f, 0.312032521f, 0.314881742f, 0.317737937f, 0.320601076f, 0.323470891f, 0.326347351f, 0.329230458f,
    0.332119942f, 0.335015774f, 0.337917864f, 0.340826035f, 0.343740195f, 0.346660197f, 0.349586099f, 0.352517605f,
    0.355454683f, 0.358397305f, 0.361345172f, 0.364298284f, 0.367256492f, 0.370219737f, 0.373187840f, 0.376160830f,
    0.379138410f, 0.382120550f, 0.385107160f, 0.388098061f, 0.391093224f, 0.394092530f, 0.397095770f, 0.400102913f,
    0.403113812f, 0.406128347f, 0.409146458f, 0.412167966f, 0.415192872f, 0.418220907f, 0.421252012f, 0.424286187f,
    0.427323043f, 0.430362761f, 0.433405131f, 0.436449945f, 0.439497173f, 0.442546725f, 0.445598423f, 0.448652089f,
    0.451707810f, 0.454765290f, 0.457824469f, 0.460885257f, 0.463947564f, 0.467011184f, 0.470075995f, 0.473142058f,
    0.476209044f, 0.479276955f, 0.482345700f, 0.485415041f, 0.488484949f, 0.491555274f, 0.494625926f, 0.497696787f,
    0.500767767f, 0.503838718f, 0.506909490f, 0.509979963f, 0.513050079f, 0.516119719f, 0.519188762f, 0.522257149f,
    0.525324583f, 0.528391123f, 0.531456649f, 0.534520805f, 0.537583768f, 0.540645361f, 0.543705344f, 0.546763718f,
    0.549820364f, 0.552875102f, 0.555927753f, 0.558978379f, 0.562026799f, 0.565072834f, 0.568116426f, 0.571157455f,
    0.574195802f, 0.577231288f, 0.580263972f, 0.583293557f, 0.586319983f, 0.589343250f, 0.592363060f, 0.595379412f,
    0.598392129f, 0.601401210f, 0.604406357f, 0.607407689f, 0.610404909f, 0.613397956f, 0.616386771f, 0.619371116f,
    0.622350991f, 0.625326276f, 0.628296852f, 0.631262541f, 0.634223282f, 0.637178957f, 0.640129447f, 0.643074691f,
    0.646014571f, 0.648948908f, 0.651877582f, 0.654800594f, 0.657717705f, 0.660628855f, 0.663534045f, 0.666432977f,
    0.669325650f, 0.672211885f, 0.675091743f, 0.677964866f, 0.680831373f, 0.683691025f, 0.686543763f, 0.689389467f,
    0.692228079f, 0.695059299f, 0.697883189f, 0.700699687f, 0.703508615f, 0.706309795f, 0.709103227f, 0.711888731f,
    0.714666367f, 0.717435837f, 0.720197082f, 0.722950041f, 0.725694537f, 0.728430569f, 0.731158078f, 0.733876765f,
    0.736586630f, 0.739287555f, 0.741979480f, 0.744662285f, 0.747335672f, 0.750000000f, 0.752654791f, 0.755300045f,
    0.757935703f, 0.760561585f, 0.763177633f, 0.765783906f, 0.768380046f, 0.770966053f, 0.773541808f, 0.776107192f,
    0.778662264f, 0.781206787f, 0.783740699f, 0.786263943f, 0.788776278f, 0.791277826f, 0.793768406f, 0.796247840f,
    0.798716187f, 0.801173210f, 0.803618848f, 0.806053042f, 0.808475673f, 0.810886681f, 0.813286066f, 0.815673411f,
    0.818048954f, 0.820412517f, 0.822764039f, 0.825103283f, 0.827430427f, 0.829745114f, 0.832047343f, 0.834337056f,
    0.836614132f, 0.838878572f, 0.841130197f, 0.843369007f, 0.845594823f, 0.847807527f, 0.850007176f, 0.852193594f,
    0.854366720f, 0.856526613f, 0.858672976f, 0.860805750f, 0.862924933f, 0.865030408f, 0.867122173f, 0.869200110f,
    0.871264100f, 0.873314023f, 0.875349879f, 0.877371550f, 0.879379034f, 0.881372213f, 0.883351088f, 0.885315359f,
    0.887265146f, 0.889200330f, 0.891120791f, 0.893026590f, 0.894917488f, 0.896793544f, 0.898654580f, 0.900500596f,
    0.902331471f, 0.904147148f, 0.905947626f, 0.907732844f, 0.909502625f, 0.911256969f, 0.912995815f, 0.914718986f,
    0.916426659f, 0.918118596f, 0.919794679f, 0.921454966f, 0.923099339f, 0.924727798f, 0.926340163f, 0.927936435f,
    0.929516673f, 0.931080580f, 0.932628334f, 0.934159756f, 0.935674727f, 0.937173307f, 0.938655436f, 0.940120935f,
    0.941569924f, 0.943002224f, 0.944417715f, 0.945816517f, 0.947198510f, 0.948563635f, 0.949911833f, 0.951243043f,
    0.952557206f, 0.953854382f, 0.955134392f, 0.956397295f, 0.957642913f, 0.958871245f, 0.960082293f, 0.961275935f,
    0.962452292f, 0.963611126f, 0.964752436f, 0.965876281f, 0.966982543f, 0.968071163f, 0.969142139f, 0.970195472f,
    0.971230984f, 0.972248793f, 0.973248720f, 0.974230826f, 0.975195050f, 0.976141334f, 0.977069676f, 0.977980018f,
    0.978872240f, 0.979746461f, 0.980602622f, 0.981440604f, 0.982260466f, 0.983062148f, 0.983845592f, 0.984610736f,
    0.985357642f, 0.986086249f, 0.986796498f, 0.987488389f, 0.988161862f, 0.988816977f, 0.989453554f, 0.990071774f,
    0.990671456f, 0.991252661f, 0.991815269f, 0.992359400f, 0.992884874f, 0.993391871f, 0.993880153f, 0.994349837f,
    0.994800925f, 0.995233297f, 0.995646954f, 0.996041954f, 0.996418238f, 0.996775746f, 0.997114599f, 0.997434676f,
    0.997735977f, 0.998018503f, 0.998282194f, 0.998527169f, 0.998753309f, 0.998960614f, 0.999149084f, 0.999318779f,
    0.999469638f, 0.999601603f, 0.999714732f, 0.999809027f, 0.999884486f, 0.999941051f, 0.999978781f, 0.999997616f,
    0.999997616f, 0.999978781f, 0.999941051f, 0.999884486f, 0.999809027f, 0.999714732f, 0.999601603f, 0.999469638f,
    0.999318779f, 0.999149084f, 0.998960614f, 0.998753309f, 0.998527169f, 0.998282194f, 0.998018503f, 0.997735977f,
    0.997434616f, 0.997114599f, 0.996775746f, 0.996418238f, 0.996041894f, 0.995646954f, 0.995233238f, 0.994800866f,
    0.994349837f, 0.993880153f, 0.993391812f, 0.992884874f, 0.992359400f, 0.991815269f, 0.991252661f, 0.990671396f,
    0.990071714f, 0.989453554f, 0.988816917f, 0.988161862f, 0.987488389f, 0.986796498f, 0.986086190f, 0.985357642f,
    0.984610677f, 0.983845532f, 0.983062148f, 0.982260466f, 0.981440663f, 0.980602622f, 0.979746461f, 0.978872180f,
    0.977979898f, 0.977069616f, 0.976141334f, 0.975194991f, 0.974230766f, 0.973248720f, 0.972248733f, 0.971230984f,
    0.970195413f, 0.969142079f, 0.968071103f, 0.966982484f, 0.965876222f, 0.964752436f, 0.963611066f, 0.962452292f,
    0.961275935f, 0.960082233f, 0.958871245f, 0.957642853f, 0.956397235f, 0.955134392f, 0.953854322f, 0.952557206f,
    0.951243043f, 0.949911833f, 0.948563576f, 0.947198510f, 0.945816517f, 0.944417715f, 0.943002105f, 0.941569865f,
    0.940120935f, 0.938655376f, 0.937173307f, 0.935674667f, 0.934159636f, 0.932628274f, 0.931080580f, 0.929516613f,
    0.927936435f, 0.926340044f, 0.924727678f, 0.923099279f, 0.921454906f, 0.919794679f, 0.918118536f, 0.916426659f,
    0.914718986f, 0.912995696f, 0.911256909f, 0.909502625f, 0.907732844f, 0.905947626f, 0.904147089f, 0.902331352f,
    0.900500536f, 0.898654580f, 0.896793485f, 0.894917488f, 0.893026471f, 0.891120791f, 0.889200270f, 0.887265086f,
    0.885315299f, 0.883350968f, 0.881372213f, 0.879378974f, 0.877371490f, 0.875349760f, 0.873314023f, 0.871264040f,
    0.869200110f, 0.867122114f, 0.865030408f, 0.862924874f, 0.860805690f, 0.858672857f, 0.856526554f, 0.854366660f,
    0.852193475f, 0.850007057f, 0.847807407f, 0.845594645f, 0.843368948f, 0.841130197f, 0.838878512f, 0.836614132f,
    0.834336996f, 0.832047284f, 0.829745054f, 0.827430367f, 0.825103402f, 0.822764039f, 0.820412636f, 0.818049014f,
    0.815673351f, 0.813285947f, 0.810886621f, 0.808475733f, 0.806052923f, 0.803618848f, 0.801173091f, 0.798716009f,
    0.796247840f, 0.793768287f, 0.791277885f, 0.788776278f, 0.786263704f, 0.783740640f, 0.781206608f, 0.778662205f,
    0.776107073f, 0.773541689f, 0.770965815f, 0.768379748f, 0.765783727f, 0.763177514f, 0.760561526f, 0.757935762f,
    0.755300164f, 0.752654850f, 0.749999940f, 0.747335851f, 0.744662166f, 0.741979480f, 0.739287496f, 0.736586452f,
    0.733876705f, 0.731157899f, 0.728430629f, 0.725694478f, 0.722950101f, 0.720197022f, 0.717435658f, 0.714666307f,
    0.711888671f, 0.709103227f, 0.706309676f, 0.703508615f, 0.700699568f, 0.697883010f, 0.695059180f, 0.692227781f,
    0.689389348f, 0.686543584f, 0.683690965f, 0.680831432f, 0.677964866f, 0.675091803f, 0.672211885f, 0.669325709f,
    0.666432977f, 0.663533866f, 0.660628915f, 0.657717586f, 0.654800594f, 0.651877522f, 0.648948908f, 0.646014452f,
    0.643074572f, 0.640129447f, 0.637178838f, 0.634223282f, 0.631262422f, 0.628296614f, 0.625326157f, 0.622350812f,
    0.619371057f, 0.616386533f, 0.613397896f, 0.610404730f, 0.607407391f, 0.604406238f, 0.601401150f, 0.598392248f,
    0.595379353f, 0.592363179f, 0.589343190f, 0.586319923f, 0.583293557f, 0.580263853f, 0.577231348f, 0.574195743f,
    0.571157277f, 0.568116367f, 0.565072656f, 0.562026739f, 0.558978260f, 0.555927753f, 0.552874923f, 0.549820125f,
    0.546763659f, 0.543705165f, 0.540645301f, 0.537583649f, 0.534520805f, 0.531456411f, 0.528390884f, 0.525324464f,
    0.522256851f, 0.519188702f, 0.516119778f, 0.513050020f, 0.509980023f, 0.506909370f, 0.503838718f, 0.500767708f,
    0.497696877f, 0.494625896f, 0.491555125f, 0.488484919f, 0.485414892f, 0.482345670f, 0.479276866f, 0.476209074f,
    0.473141968f, 0.470075846f, 0.467011094f, 0.463947356f, 0.460885227f, 0.457824320f, 0.454765022f, 0.451707661f,
    0.448651880f, 0.445598274f, 0.442546487f, 0.439497083f, 0.436449736f, 0.433405042f, 0.430362821f, 0.427323043f,
    0.424286187f, 0.421252012f, 0.418220997f, 0.415192842f, 0.412167907f, 0.409146488f, 0.406128287f, 0.403113842f,
    0.400102824f, 0.397095561f, 0.394092441f, 0.391093075f, 0.388098031f, 0.385107011f, 0.382120520f, 0.379138261f,
    0.376160562f, 0.373187780f, 0.370219529f, 0.367256403f, 0.364298075f, 0.361345083f, 0.358397096f, 0.355454445f,
    0.352517486f, 0.349586070f, 0.346660346f, 0.343740165f, 0.340826124f, 0.337917864f, 0.335015684f, 0.332119972f,
    0.329230368f, 0.326347440f, 0.323470831f, 0.320600867f, 0.317737937f, 0.314881623f, 0.312032521f, 0.309190273f,
    0.306355476f, 0.303527743f, 0.300707430f, 0.297894835f, 0.295089662f, 0.292292446f, 0.289502859f, 0.286721408f,
    0.283947766f, 0.281182289f, 0.278425276f, 0.275676429f, 0.272936255f, 0.270204663f, 0.267481476f, 0.264767289f,
    0.262061775f, 0.259365439f, 0.256677985f, 0.253999949f, 0.251330912f, 0.248671293f, 0.246021360f, 0.243380815f,
    0.240750134f, 0.238129050f, 0.235518038f, 0.232916802f, 0.230325639f, 0.227744848f, 0.225174099f, 0.222613961f,
    0.220064074f, 0.217524737f, 0.214996278f, 0.212478340f, 0.209971458f, 0.207475334f, 0.204990417f, 0.202516466f,
    0.200053900f, 0.197602659f, 0.195162654f, 0.192734301f, 0.190317363f, 0.187912315f, 0.185518831f, 0.183137208f,
    0.180767745f, 0.178410143f, 0.176064819f, 0.173731565f, 0.171410620f, 0.169102222f, 0.166806161f, 0.164522827f,
    0.162251979f, 0.159994036f, 0.157748759f, 0.155516386f, 0.153297186f, 0.151090890f, 0.148897916f, 0.146718025f,
    0.144551635f, 0.142398477f, 0.140258819f, 0.138132870f, 0.136020601f, 0.133922070f, 0.131837159f, 0.129766136f,
    0.127709270f, 0.125666261f, 0.123637527f, 0.121622860f, 0.119622618f, 0.117636532f, 0.115664899f, 0.113707930f,
    0.111765355f, 0.109837592f, 0.107924402f, 0.106026143f, 0.104142606f, 0.102274001f, 0.100420535f, 0.098582000f,
    0.096758753f, 0.094950587f, 0.093157679f, 0.091380268f, 0.089618146f, 0.087871611f, 0.086140513f, 0.084425151f,
    0.082725465f, 0.081041396f, 0.079373270f, 0.077720851f, 0.076084524f, 0.074464023f, 0.072859734f, 0.071271420f,
    0.069699287f, 0.068143487f, 0.066603899f, 0.065080732f, 0.063573867f, 0.062083483f, 0.060609728f, 0.059152424f,
    0.057711869f, 0.056287885f, 0.054880768f, 0.053490311f, 0.052116692f, 0.050760090f, 0.049420327f, 0.048097670f,
    0.046791971f, 0.045503438f, 0.044231951f, 0.042977780f, 0.041740835f, 0.040521085f, 0.039318740f, 0.038133711f,
    0.036966085f, 0.035816044f, 0.034683406f, 0.033568412f, 0.032470912f, 0.031391144f, 0.030328959f, 0.029284507f,
    0.028257906f, 0.027249008f, 0.026257992f, 0.025284797f, 0.024329603f, 0.023392260f, 0.022472888f, 0.021571606f,
    0.020688325f, 0.019823164f, 0.018976063f, 0.018147111f, 0.017336398f, 0.016543806f, 0.015769541f, 0.015013546f,
    0.014275819f, 0.013556361f, 0.012855262f, 0.012172610f, 0.011508286f, 0.010862440f, 0.010235012f, 0.009626091f,
    0.009035647f, 0.008463711f, 0.007910341f, 0.007375509f, 0.006859303f, 0.006361663f, 0.005882680f, 0.005422294f,
    0.004980564f, 0.004557550f, 0.004153192f, 0.003767580f, 0.003400624f, 0.003052443f, 0.002723008f, 0.002412319f,
    0.002120405f, 0.001847267f, 0.001592964f, 0.001357436f, 0.001140684f, 0.000942796f, 0.000763685f, 0.000603467f,
    0.000462025f, 0.000339478f, 0.000235736f, 0.000150889f, 0.000084877f, 0.000037730f, 0.000009418f, 0.000000000f,
};
