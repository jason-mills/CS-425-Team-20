using _3CPO_v2.ViewModel;

namespace _3CPO_v2;

public partial class SettingsPage : ContentPage
{
	public SettingsPage(SettingsViewModel vm)
	{
		InitializeComponent();
        BindingContext = vm;
	}
}